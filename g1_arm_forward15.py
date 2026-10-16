#!/usr/bin/env python3
"""G1 23DOF staged two-shoulder forward-angle check on rt/arm_sdk.

WARNING: UNVERIFIED HARDWARE CODE. No locomotion or leg joint commands.
Uses the current standing FSM, confirms FSM continuously via a read-only RPC,
and aborts progression if measured shoulder movement does not track.
The operator must provide an independent physical E-stop and overhead protection.
"""
import argparse
import csv
import json
import math
import os
import signal
import threading
import time

from unitree_sdk2py.core.channel import (ChannelFactoryInitialize, ChannelPublisher,
                                        ChannelSubscriber)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

ARMS = (15, 16, 17, 18, 19, 22, 23, 24, 25, 26)
SHOULDERS = (15, 22)
NAMES = {15:'left_shoulder_pitch', 16:'left_shoulder_roll', 17:'left_shoulder_yaw',
         18:'left_elbow', 19:'left_wrist_roll', 22:'right_shoulder_pitch',
         23:'right_shoulder_roll', 24:'right_shoulder_yaw', 25:'right_elbow',
         26:'right_wrist_roll'}
WEIGHT_INDEX = 29
HZ = 50.0
DT = 1.0 / HZ
STALE_S = 0.5
FORWARD_SIGN = -1.0  # G1 23DOF rev1 URDF: shoulder pitch axis +Y, forward X
SHOULDER_PITCH_LIMIT = (-3.0892, 2.6704)  # official rev1 URDF


def smooth(t):
    t = max(0., min(1., t))
    return t*t*(3.-2.*t)


def deg(radians):
    return math.degrees(radians)


def q_of(state, i):
    if len(state.motor_state) <= i:
        raise RuntimeError('rt/lowstate joint array too short')
    q = float(state.motor_state[i].q)
    if not math.isfinite(q):
        raise RuntimeError('non-finite joint feedback index {}'.format(i))
    return q


class StateReader:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = None
        self.at = 0.0
        self.sub = ChannelSubscriber('rt/lowstate', LowState_)
        self.sub.Init(self._cb, 10)

    def _cb(self, state):
        with self.lock:
            self.state = state
            self.at = time.monotonic()

    def get(self):
        with self.lock:
            state, stamp = self.state, self.at
        if state is None or time.monotonic()-stamp > STALE_S:
            raise RuntimeError('rt/lowstate stale or missing')
        return state

    def wait(self, seconds=5.0):
        limit = time.monotonic()+seconds
        while time.monotonic() < limit:
            try:
                return self.get()
            except RuntimeError:
                time.sleep(.05)
        raise RuntimeError('no fresh rt/lowstate')


def parse_rpc(reply):
    if not isinstance(reply, (tuple, list)) or len(reply) != 2:
        raise RuntimeError('unexpected FSM RPC format')
    rc, payload = reply
    if rc != 0:
        raise RuntimeError('FSM RPC failure code {}'.format(rc))
    if isinstance(payload, str):
        payload = json.loads(payload)
    if isinstance(payload, dict):
        payload = payload.get('data')
    if payload is None:
        raise RuntimeError('FSM RPC data missing')
    number = int(payload)
    if isinstance(payload, float) and not payload.is_integer():
        raise RuntimeError('non-integer FSM data')
    return number


class FsmReader:
    def __init__(self):
        self.cli = LocoClient()
        self.cli.SetTimeout(.7)
        self.cli.Init()

    def fsm_id(self):
        method = getattr(self.cli, 'GetFsmId', None)
        if callable(method):
            return parse_rpc(method())
        method = getattr(self.cli, '_Call', None)
        if not callable(method):
            raise RuntimeError('SDK has neither GetFsmId nor read-only _Call')
        return parse_rpc(method(7001, '{}'))


class FsmWatch:
    def __init__(self, fsm_reader, expected):
        self.reader = fsm_reader
        self.expected = expected
        self.lock = threading.Lock()
        self.stop_evt = threading.Event()
        self.last_ok = 0.0
        self.problem = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        while not self.stop_evt.is_set():
            try:
                fsm = self.reader.fsm_id()
                with self.lock:
                    if fsm != self.expected:
                        self.problem = 'FSM changed: {} (expected {})'.format(fsm, self.expected)
                        return
                    self.last_ok = time.monotonic()
            except Exception as exc:
                with self.lock:
                    if self.last_ok and time.monotonic() - self.last_ok > 2.5:
                        self.problem = 'FSM RPC unavailable: {}'.format(exc)
                        return
            self.stop_evt.wait(.8)

    def check(self):
        with self.lock:
            problem, last = self.problem, self.last_ok
        if problem:
            raise RuntimeError(problem)
        if not last or time.monotonic()-last > 2.5:
            raise RuntimeError('FSM could not be monitored (fail closed)')

    def close(self):
        self.stop_evt.set()
        self.thread.join(timeout=1.5)


class ForwardTest:
    def __init__(self, reader, fsm_watch, fsm_id, csv_path, max_degrees, hold):
        self.reader = reader
        self.fsm_watch = fsm_watch
        self.fsm_id = fsm_id
        self.max_degrees = max_degrees
        self.hold = hold
        state = self.reader.get()
        self.mode_machine = int(state.mode_machine)
        self.original = {i:q_of(state,i) for i in ARMS}
        self.reference = dict(self.original)
        self.command_q = dict(self.original)
        self.weight = 0.
        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.mode_machine
        self.pub = ChannelPublisher('rt/arm_sdk', LowCmd_)
        self.pub.Init()
        self.csv_f = open(csv_path, 'w', newline='', buffering=1)
        self.csv_writer = csv.writer(self.csv_f)
        self.csv_writer.writerow(['unix_time','phase','fsm_id_start','mode_machine','weight',
                                  'joint_id','joint_name','target_rad','actual_rad','error_rad'])
        for i, motor in enumerate(self.msg.motor_cmd):
            motor.mode = 1
            motor.q = q_of(state, i) if i < len(state.motor_state) else 0.
            motor.dq = 0.
            motor.tau = 0.
            motor.kp = 0.
            motor.kd = 0.
        for i in ARMS:
            motor = self.msg.motor_cmd[i]
            motor.kp = 20. if i in (19,26) else 35.
            motor.kd = 1. if i in (19,26) else 1.5
        self.msg.motor_cmd[WEIGHT_INDEX].q = 0.
        self.outcome = 'NOT_RUN'

    def emit(self, phase, targets, weight, verified=True):
        # Check sensors and modes before sending any positive-weight command.
        if verified:
            self.fsm_watch.check()
            state = self.reader.get()
            if int(state.mode_machine) != self.mode_machine:
                raise RuntimeError('DDS mode_machine changed: {} -> {}'.format(
                    self.mode_machine, state.mode_machine))
            for i in ARMS:
                actual = q_of(state, i)
                if abs(deg(actual-self.reference[i])) > (22. if i in SHOULDERS else 6.):
                    raise RuntimeError('joint {} unexpected excursion {:.2f}deg'.format(
                        i, deg(actual-self.reference[i])))
        else:
            try:
                state = self.reader.get()
            except Exception:
                state = None
        for i in ARMS:
            self.msg.motor_cmd[i].q = targets[i]
        self.msg.motor_cmd[WEIGHT_INDEX].q = weight
        self.msg.crc = self.crc.Crc(self.msg)
        self.pub.Write(self.msg)
        self.command_q = dict(targets)
        self.weight = weight
        if state is not None:
            ts = time.time()
            for i in ARMS:
                self.csv_writer.writerow([ts, phase, self.fsm_id, self.mode_machine,
                                          round(weight,4),i,NAMES[i],
                                          targets[i],q_of(state,i),targets[i]-q_of(state,i)])

    def ramp(self, name, seconds, destination, weight_start=None, weight_end=None):
        start_q = dict(self.command_q)
        w0 = self.weight if weight_start is None else weight_start
        w1 = w0 if weight_end is None else weight_end
        t0 = time.monotonic()
        n = 0
        print('[{}] {:.1f}s'.format(name,seconds), flush=True)
        while True:
            elapsed = time.monotonic()-t0
            frac = smooth(1.0 if seconds <= 0 else elapsed/seconds)
            target = {j:start_q[j]+(destination[j]-start_q[j])*frac for j in ARMS}
            w = w0 + (w1-w0)*frac
            self.emit(name,target,w)
            if n % 25 == 0:
                s=self.reader.get()
                print('  w={:.2f} L cmd={:+5.1f} actual={:+5.1f} | R cmd={:+5.1f} actual={:+5.1f}'.format(
                    w,deg(target[15]-self.reference[15]),deg(q_of(s,15)-self.reference[15]),
                    deg(target[22]-self.reference[22]),deg(q_of(s,22)-self.reference[22])),flush=True)
            if elapsed >= seconds:
                break
            n+=1
            time.sleep(max(0.,t0+n*DT-time.monotonic()))

    def settle(self, name, seconds):
        self.ramp(name,seconds,dict(self.command_q))

    def evaluate_stage(self, stage_deg):
        state = self.reader.get()
        for i in SHOULDERS:
            move = deg(q_of(state,i)-self.reference[i])*FORWARD_SIGN
            err = abs(deg(self.command_q[i]-q_of(state,i)))
            print('  [gate] {} expected forward {:.1f}deg, actual {:+.2f}deg, err {:.2f}deg'.format(
                NAMES[i], stage_deg, move, err),flush=True)
            if move < .70*stage_deg or err > max(2.0,.30*stage_deg):
                raise RuntimeError('TRACKING_FAIL joint={} step={}deg actual={:.2f}deg err={:.2f}deg; refusing larger step'.format(
                    i, stage_deg, move, err))

    def safe_finish(self):
        # Never rely on this software routine as an E-stop.
        try:
            healthy = bool(self.fsm_watch)
            self.fsm_watch.check()
            state = self.reader.get()
            healthy = healthy and (int(state.mode_machine) == self.mode_machine)
        except Exception:
            healthy = False
        if healthy and self.weight > 0:
            try:
                print('[RETURN] slowly returning to post-takeover reference',flush=True)
                self.ramp('return', 3., dict(self.reference))
            except Exception as exc:
                print('WARNING: return trajectory unavailable: {}'.format(exc),flush=True)
        if self.weight > 0:
            print('[RELEASE] decreasing arm_sdk weight to zero (best effort)',flush=True)
            q = dict(self.command_q)
            w = self.weight
            try:
                for n in range(61):
                    self.emit('release',q,w*(1.-smooth(n/60.)),verified=False)
                    time.sleep(DT)
            except Exception as exc:
                print('WARNING: software handback not confirmed: {}'.format(exc),flush=True)

    def execute(self):
        old_handler=signal.getsignal(signal.SIGINT)
        try:
            self.ramp('takeover', 2., dict(self.original), weight_start=0.,weight_end=1.)
            self.settle('takeover_settle', .7)
            state=self.reader.get()
            drift={i:deg(q_of(state,i)-self.original[i]) for i in ARMS}
            for i in ARMS:
                if abs(drift[i]) > 4.:
                    raise RuntimeError('takeover drift too large, {} {:+.2f}deg'.format(NAMES[i],drift[i]))
            print('Post-takeover shoulder drift: L={:+.2f}deg R={:+.2f}deg'.format(
                drift[15],drift[22]),flush=True)
            # Only compute forward-offset relative to real, post-takeover feedback.
            self.reference = {i:q_of(state,i) for i in ARMS}
            self.ramp('reanchor', .6, dict(self.reference))
            stages=[x for x in (3.,5.,10.,15.) if x <= self.max_degrees]
            if self.max_degrees not in stages:
                stages.append(self.max_degrees)
            for goal_deg in stages:
                dest=dict(self.reference)
                for joint in SHOULDERS:
                    dest[joint]+=FORWARD_SIGN*math.radians(goal_deg)
                self.ramp('forward_{:g}deg'.format(goal_deg),2.,dest)
                self.settle('gate_hold',.8)
                self.evaluate_stage(goal_deg)
            self.settle('final_hold',self.hold)
            self.outcome='PASS_POSITION_TRACKING'
        finally:
            # A second Ctrl+C during handback must not interrupt the routine.
            signal.signal(signal.SIGINT,lambda *_: print('Use physical remote E-stop if needed; handback in progress',flush=True))
            try:
                self.safe_finish()
            finally:
                signal.signal(signal.SIGINT,old_handler)
                self.csv_f.close()


def main():
    parser=argparse.ArgumentParser(description='G1 23DOF staged dual shoulder forward test, no walking')
    parser.add_argument('--iface',required=True)
    parser.add_argument('--probe',action='store_true')
    parser.add_argument('--test',action='store_true')
    parser.add_argument('--expect-fsm',type=int,default=500)
    parser.add_argument('--degrees',type=float,default=15.,help='maximum offset in degrees (3..15), staged')
    parser.add_argument('--hold',type=float,default=3.)
    parser.add_argument('--csv',default='fsm500_both_forward15.csv')
    args=parser.parse_args()
    if args.probe == args.test:
        parser.error('choose exactly one of --probe or --test')
    if not (3. <= args.degrees <= 15.):
        parser.error('--degrees must be within 3..15')
    if not (1. <= args.hold <= 10.):
        parser.error('--hold must be within 1..10 sec')

    ChannelFactoryInitialize(0,args.iface)
    reader=StateReader()
    state=reader.wait()
    print('DDS mode_machine:', int(state.mode_machine))
    print('G1 23DOF arm joint state:')
    for j in ARMS:
        print('  {:2d} {:24s} q={:+.4f} rad dq={:+.4f} rad/s'.format(
            j, NAMES[j],q_of(state,j),float(state.motor_state[j].dq)))
    client=FsmReader()
    fsm=client.fsm_id()
    print('Read-only FSM ID:',fsm)
    if args.probe:
        print('PROBE_OK: no control published')
        return
    if fsm != args.expect_fsm:
        raise RuntimeError('FSM mismatch {} != {}; refusing to move'.format(fsm,args.expect_fsm))
    if int(state.mode_machine)!=4:
        raise RuntimeError('DDS mode_machine is not expected 4; verify model/firmware')
    for j in ARMS:
        if abs(float(state.motor_state[j].dq)) > .15:
            raise RuntimeError('joint {} moving before test'.format(j))
    for j in SHOULDERS:
        target=q_of(state,j)+FORWARD_SIGN*math.radians(args.degrees)
        if not SHOULDER_PITCH_LIMIT[0]+.05 < target < SHOULDER_PITCH_LIMIT[1]-.05:
            raise RuntimeError('shoulder target outside URDF joint limit {}'.format(j))

    print('\nWARNING: UNVERIFIED PHYSICAL ROBOT TEST. Arms can move abruptly.')
    print('Requires overhead fall restraint, space clear, free arms, and second operator holding independent E-stop.')
    print('NO object held; NO walking; NO other arm_sdk publisher; DO NOT manually push the arms.')
    print('Targets: BOTH shoulder pitch joints {} decrease from post-takeover feedback.'.format(SHOULDERS))
    print('Steps: 3 -> 5 -> 10 -> 15 degrees, abort progression if feedback fails.')
    if input('Type FORWARD15 to enable (otherwise cancel): ').strip() != 'FORWARD15':
        print('Cancelled; no motor commands published')
        return
    watch=FsmWatch(client,args.expect_fsm)
    watch.start()
    try:
        # Wait for the monitoring thread to receive a real, current FSM sample.
        time.sleep(1.)
        watch.check()
        test=ForwardTest(reader,watch,fsm,args.csv,args.degrees,args.hold)
        test.execute()
        print('Result:',test.outcome)
        print('CSV:',os.path.abspath(args.csv))
    finally:
        watch.close()

if __name__=='__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted. Confirm state with physical remote before next test.')
    except Exception as exc:
        print('TEST_FAILED:',exc,flush=True)
        raise
