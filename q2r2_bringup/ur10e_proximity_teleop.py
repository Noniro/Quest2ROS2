#!/usr/bin/env python3
"""
UR10e proximity-based clutch teleop controller.

Engagement:
  Press A (right controller) to toggle teleop ON / OFF.
  On every engage the Quest2 anchor is reset to the current controller
  position → robot TCP stays put, no sudden jumps.

Disengagement:
  A button again, B button (emergency), or too many consecutive IK failures.

Visualization (RViz, topic /teleop_markers, frame base_link):
  - Large sphere ABOVE robot: green = ENGAGED, red = IDLE
  - TCP sphere + XYZ axes arrows (current robot TCP orientation)
  - Quest2 hand sphere + XYZ axes arrows (mapped orientation when engaged)
  - Text label: state + joint-delta info

Coordinate mapping Quest2 → robot base_link:
  Robot X (fwd)  = -Quest Z
  Robot Y (left) = -Quest X
  Robot Z (up)   =  Quest Y

Home service:
  ros2 service call /go_home std_srvs/srv/Trigger
  → resets robot to saved home pose and disengages.

Sim mode  (sim_mode:=true):  publishes /joint_states — RViz animates model.
HW mode   (sim_mode:=false): publishes /forward_position_controller/commands
                              when ENGAGED only.
"""

import os
import tempfile
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from quest2ros.msg import OVR2ROSInputs

from ament_index_python.packages import get_package_share_directory
import xacro
import ikpy.chain
import xml.etree.ElementTree as ET


# ── State IDs ──────────────────────────────────────────────────────────────
IDLE    = 0
ENGAGED = 1
HOMING  = 2
_NAME   = {IDLE: 'IDLE', ENGAGED: 'ENGAGED', HOMING: 'HOMING'}

# ── Saved home pose (jogged on pendant 2026-06-16) ─────────────────────────
# Base 0°, Shoulder -90°, Elbow 145°, Wrist1 -55°, Wrist2 90°, Wrist3 0°
UR_HOME = {
    'shoulder_pan_joint':  0.0,
    'shoulder_lift_joint': -1.5708,
    'elbow_joint':          2.5307,
    'wrist_1_joint':       -0.9599,
    'wrist_2_joint':        1.5708,
    'wrist_3_joint':        0.0,
}

# ── Axis remapping: Quest2 → robot base_link ────────────────────────────────
# In the Q2ROS2 app the controller X axis is the user's depth (back/forth),
# Y is up/down, Z is left/right.
# Robot base_link: X = forward, Y = left, Z = up.
#
# If left/right or fwd/back feel inverted, negate the corresponding row.
R_Q2R = np.array([
    [ 1.,  0.,  0.],   # Robot X (fwd)  = +Quest X (user fwd/back)
    [ 0.,  0., -1.],   # Robot Y (left) = -Quest Z (user left/right)
    [ 0.,  1.,  0.],   # Robot Z (up)   = +Quest Y (user up/down)
])

# Map from axis token -> (column index, sign).  Used to parse axis-map strings
# like "x,-z,y" into a 3x3 mapping matrix (each output row = one input axis).
_AXIS_TOKEN = {
    'x': (0,  1.0), '+x': (0,  1.0), '-x': (0, -1.0),
    'y': (1,  1.0), '+y': (1,  1.0), '-y': (1, -1.0),
    'z': (2,  1.0), '+z': (2,  1.0), '-z': (2, -1.0),
}


def _parse_axis_map(spec: str) -> np.ndarray:
    """Parse 'x,-z,y' into a 3x3 signed-permutation matrix.

    Row i of the result picks which Quest axis (and sign) becomes robot axis i.
    E.g. 'x,-z,y' -> robot_x=+quest_x, robot_y=-quest_z, robot_z=+quest_y.
    """
    parts = [p.strip().lower() for p in spec.split(',')]
    if len(parts) != 3:
        raise ValueError(f'axis map needs 3 comma-separated tokens, got: {spec!r}')
    M = np.zeros((3, 3))
    for row, tok in enumerate(parts):
        if tok not in _AXIS_TOKEN:
            raise ValueError(f'bad axis token {tok!r} in {spec!r}')
        col, sign = _AXIS_TOKEN[tok]
        M[row, col] = sign
    return M


def _quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z,   2*x*y - 2*z*w,       2*x*z + 2*y*w],
        [2*x*y + 2*z*w,       1 - 2*x*x - 2*z*z,   2*y*z - 2*x*w],
        [2*x*z - 2*y*w,       2*y*z + 2*x*w,       1 - 2*x*x - 2*y*y],
    ])


def _derive_chain_spec(urdf_xml, base='base_link', tip='tool0'):
    root = ET.fromstring(urdf_xml)
    children: dict = {}
    for j in root.findall('joint'):
        p = j.find('parent').get('link')
        c = j.find('child').get('link')
        children.setdefault(p, []).append((j.get('name'), j.get('type'), c))

    def dfs(link, path):
        if link == tip:
            return path
        for jn, jt, ch in children.get(link, []):
            found = dfs(ch, path + [(jn, jt, ch)])
            if found is not None:
                return found
        return None

    path = dfs(base, [])
    if path is None:
        raise RuntimeError(f'No kinematic path {base} → {tip} in URDF')
    elements = [base]
    mask = [False]
    joint_names = []
    for jn, jt, ch in path:
        elements += [jn, ch]
        act = jt in ('revolute', 'continuous', 'prismatic')
        mask.append(act)
        if act:
            joint_names.append(jn)
    return elements, mask, joint_names


class UR10eProximityTeleop(Node):

    def __init__(self):
        super().__init__('ur10e_proximity_teleop')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('sim_mode',             True)
        self.declare_parameter('scale_factor',         1.0)
        self.declare_parameter('publish_rate',         30.0)
        # Per-tick joint rate limit (rad/tick). IK steps are CLAMPED to this so
        # fast hand moves make the robot trail smoothly & catch up — instead of
        # sending a violating step or disengaging. At publish_rate Hz the joint
        # speed cap is max_joint_delta * publish_rate rad/s.
        self.declare_parameter('max_joint_delta',      0.05)
        # Sanity guard: a single-tick raw IK jump bigger than this means an
        # IK/singularity flip (not real motion) — hold rather than chase it.
        self.declare_parameter('max_ik_jump',          1.0)
        self.declare_parameter('sync_rate_limit',      True)   # scale all joints together (quiet) vs per-joint clip (staggered)
        # Output smoothing — exponential moving average (1st-order low-pass).
        # published = alpha*raw + (1-alpha)*previous. 1.0 = off (raw, jittery),
        # lower = smoother (less tremor/noise, slightly more lag). Live-tunable.
        self.declare_parameter('smoothing_alpha',      0.5)
        self.declare_parameter('disengage_fail_count', 15)
        self.declare_parameter('home_speed_deg_s',     8.0)   # °/s toward home
        # Position + orientation axis maps (Quest -> robot), tunable live.
        # Each token = which Quest axis (with sign) becomes that robot axis.
        self.declare_parameter('pos_axis_map',         'x,y,z')
        self.declare_parameter('ori_axis_map',         'x,y,z')
        # World-frame: TCP mirrors hand rotation in the room (recommended).
        self.declare_parameter('world_frame_ori',      True)
        self.declare_parameter('joint_states_topic',   '/joint_states')
        self.declare_parameter('fwd_pos_topic',        '/forward_position_controller/commands')

        self.sim_mode   = self.get_parameter('sim_mode').value
        self.scale      = self.get_parameter('scale_factor').value
        self.rate       = self.get_parameter('publish_rate').value
        self.max_delta  = self.get_parameter('max_joint_delta').value
        self.max_ik_jump = self.get_parameter('max_ik_jump').value
        self.sync_rl    = bool(self.get_parameter('sync_rate_limit').value)
        self.smooth_a   = float(self.get_parameter('smoothing_alpha').value)
        self.fail_max   = self.get_parameter('disengage_fail_count').value
        home_deg_s      = self.get_parameter('home_speed_deg_s').value
        self.home_step  = np.radians(home_deg_s) / self.rate   # rad per tick
        js_topic        = self.get_parameter('joint_states_topic').value
        fwdp_topic      = self.get_parameter('fwd_pos_topic').value

        # Position + orientation mapping matrices (parsed from axis-map strings)
        self.R_pos      = self._safe_parse(self.get_parameter('pos_axis_map').value, 'x,y,z')
        self.R_ori      = self._safe_parse(self.get_parameter('ori_axis_map').value, 'x,y,z')
        self.world_frame = self.get_parameter('world_frame_ori').value
        self.add_on_set_parameters_callback(self._on_param_change)

        # ── IK chain ────────────────────────────────────────────────────
        self._init_chain()
        self.current_joints = {n: UR_HOME.get(n, 0.0) for n in self.joint_names}

        # ── State machine ────────────────────────────────────────────────
        self.state      = IDLE
        self.fail_count = 0

        # Engagement anchor (set on every A-press engage)
        self.quest_anchor     = None
        self.tcp_anchor       = None
        self.rot_anchor       = None
        self.quest_rot_anchor = None

        # Smoothed output command (EMA state); reset on each engage.
        self.cmd_filtered   = None

        self.last_quest_msg = None
        self.js_received    = self.sim_mode   # sim: current_joints already valid

        # A / B button edge detection
        self.a_prev    = False
        self.b_prev    = False
        self.a_pressed = False
        self.b_pressed = False

        # ── Subscriptions ───────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, '/q2r_right_hand_pose', self._quest_pose_cb, 10)
        self.create_subscription(
            OVR2ROSInputs, '/q2r_right_hand_inputs', self._inputs_cb, 10)
        if not self.sim_mode:
            self.create_subscription(
                JointState, js_topic, self._joint_state_cb, qos_profile_sensor_data)

        # ── Publishers ──────────────────────────────────────────────────
        self.marker_pub = self.create_publisher(MarkerArray, '/teleop_markers', 10)
        if self.sim_mode:
            self.js_pub = self.create_publisher(JointState, js_topic, 10)
        else:
            self.cmd_pub = self.create_publisher(Float64MultiArray, fwdp_topic, 10)

        # ── Services ────────────────────────────────────────────────────
        self.create_service(Trigger, 'go_home', self._go_home_cb)

        self.create_timer(1.0 / self.rate, self._tick)
        mode = 'SIM' if self.sim_mode else 'HW'
        self.get_logger().info(
            f'[ProximityTeleop] Ready ({mode}).  '
            f'A = engage/disengage.  B = emergency stop.  '
            f'max_delta: {np.degrees(self.max_delta):.1f}°/step  '
            f'Home: ros2 service call /go_home std_srvs/srv/Trigger')

    # ── IK chain init ────────────────────────────────────────────────────
    def _init_chain(self):
        pkg = get_package_share_directory('ur_description')
        xacro_f = os.path.join(pkg, 'urdf', 'ur.urdf.xacro')
        doc = xacro.process_file(xacro_f, mappings={'ur_type': 'ur10e', 'name': 'ur'})
        urdf_xml = doc.toxml()
        with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
            f.write(urdf_xml)
            self._urdf_path = f.name
        elements, mask, self.joint_names = _derive_chain_spec(urdf_xml)
        self.active_idx = [i for i, a in enumerate(mask) if a]
        self.chain = ikpy.chain.Chain.from_urdf_file(
            self._urdf_path, base_elements=elements, active_links_mask=mask)
        self.get_logger().info(f'IK chain: {self.joint_names}')

    # ── Orientation mapping (body- or world-frame) ───────────────────────
    def _map_rotation(self, q_rot):
        """Map current Quest orientation to a robot TCP target rotation,
        anchored at engagement.

        world_frame=True  : TCP mirrors how the hand rotates in the room
                            (extrinsic / left-multiply).
        world_frame=False : TCP rotates about its own axes (intrinsic).
        """
        if self.world_frame:
            R_rel_world = q_rot @ self.quest_rot_anchor.T          # extrinsic
            R_rel_rob   = self.R_ori @ R_rel_world @ self.R_ori.T
            return R_rel_rob @ self.rot_anchor
        else:
            R_rel     = self.quest_rot_anchor.T @ q_rot            # intrinsic
            R_rel_rob = self.R_ori @ R_rel @ self.R_ori.T
            return self.rot_anchor @ R_rel_rob

    # ── Homing step (one tick toward UR_HOME) ────────────────────────────
    def _step_homing(self):
        diffs = {nm: UR_HOME.get(nm, 0.0) - self.current_joints[nm]
                 for nm in self.joint_names}
        max_diff = max(abs(d) for d in diffs.values())
        if max_diff < 1e-4:
            for nm in self.joint_names:
                self.current_joints[nm] = UR_HOME.get(nm, 0.0)
            self.state = IDLE
            self.get_logger().info('[go_home] Reached home position.')
            return
        if self.sync_rl:
            # synchronized: farthest joint moves at home_step, the rest scale so
            # all finish together — no staggered per-joint snaps (quiet).
            f = min(1.0, self.home_step / max_diff)
            for nm in self.joint_names:
                self.current_joints[nm] += diffs[nm] * f
        else:
            # legacy: every joint at home_step, unsynchronized finish + snap
            for nm in self.joint_names:
                d = diffs[nm]
                if abs(d) < 1e-4:
                    self.current_joints[nm] = UR_HOME.get(nm, 0.0)
                else:
                    self.current_joints[nm] += float(np.clip(d, -self.home_step, self.home_step))

    # ── FK helper ────────────────────────────────────────────────────────
    def _fk(self):
        q = [0.0] * len(self.chain.links)
        for idx, nm in zip(self.active_idx, self.joint_names):
            q[idx] = self.current_joints[nm]
        T = self.chain.forward_kinematics(q)
        return T[:3, 3].copy(), T[:3, :3].copy()

    def _safe_parse(self, spec, fallback):
        try:
            return _parse_axis_map(spec)
        except ValueError as e:
            self.get_logger().error(f'bad axis map ({e}); using {fallback}')
            return _parse_axis_map(fallback)

    # ── Live parameter updates ───────────────────────────────────────────
    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name in ('ori_axis_map', 'pos_axis_map'):
                try:
                    M = _parse_axis_map(p.value)
                except ValueError as e:
                    self.get_logger().error(f'[{p.name}] rejected: {e}')
                    return SetParametersResult(successful=False, reason=str(e))
                if p.name == 'ori_axis_map':
                    self.R_ori = M
                else:
                    self.R_pos = M
                self.get_logger().info(f'[{p.name}] -> {p.value}')
            elif p.name == 'world_frame_ori':
                self.world_frame = bool(p.value)
                self.get_logger().info(f'[world_frame_ori] -> {p.value}')
            elif p.name == 'scale_factor':
                self.scale = p.value
                self.get_logger().info(f'[scale_factor] -> {p.value}')
            elif p.name == 'max_joint_delta':
                self.max_delta = p.value
                self.get_logger().info(
                    f'[max_joint_delta] -> {p.value} '
                    f'(~{np.degrees(p.value * self.rate):.0f}°/s cap)')
            elif p.name == 'max_ik_jump':
                self.max_ik_jump = p.value
                self.get_logger().info(f'[max_ik_jump] -> {p.value}')
            elif p.name == 'smoothing_alpha':
                a = float(p.value)
                if not 0.0 < a <= 1.0:
                    self.get_logger().error(
                        f'[smoothing_alpha] rejected: {a} (must be in (0, 1])')
                    return SetParametersResult(
                        successful=False, reason='smoothing_alpha must be in (0, 1]')
                self.smooth_a = a
                self.get_logger().info(
                    f'[smoothing_alpha] -> {a} '
                    f'({"off" if a >= 1.0 else "smoothing on"})')
            elif p.name == 'sync_rate_limit':
                self.sync_rl = bool(p.value)
                self.get_logger().info(
                    f'[sync_rate_limit] -> {self.sync_rl} '
                    f'({"synchronized" if self.sync_rl else "per-joint clip (legacy)"})')
        return SetParametersResult(successful=True)

    # ── Services ─────────────────────────────────────────────────────────
    def _go_home_cb(self, request, response):
        if not self.js_received:
            response.success = False
            response.message = ('No /joint_states yet — refusing to home '
                                '(would risk a jump from default pose).')
            self.get_logger().warn('[go_home] refused: no joint feedback yet')
            return response
        self.state        = HOMING
        self.quest_anchor = None
        self.get_logger().info(
            f'[go_home] Homing at {np.degrees(self.home_step * self.rate):.1f}°/s ...')
        response.success = True
        response.message = 'Moving to home pose.'
        return response

    # ── Callbacks ────────────────────────────────────────────────────────
    def _quest_pose_cb(self, msg: PoseStamped):
        self.last_quest_msg = msg

    def _inputs_cb(self, msg: OVR2ROSInputs):
        # Detect rising edges so hold doesn't retrigger
        a_now = bool(msg.button_lower)
        b_now = bool(msg.button_upper)
        if a_now and not self.a_prev:
            self.a_pressed = True
        if b_now and not self.b_prev:
            self.b_pressed = True
        self.a_prev = a_now
        self.b_prev = b_now

    def _joint_state_cb(self, msg: JointState):
        # While we are actively driving the robot (ENGAGED or HOMING) we own
        # current_joints — don't let feedback overwrite the commanded trajectory.
        if self.state in (ENGAGED, HOMING):
            return
        for nm, pos in zip(msg.name, msg.position):
            if nm in self.current_joints:
                self.current_joints[nm] = pos
        self.js_received = True

    # ── Main tick ────────────────────────────────────────────────────────
    def _tick(self):
        tcp_pos, tcp_rot = self._fk()
        prev_state = self.state

        # Handle B (emergency stop) — always processed
        if self.b_pressed:
            self.b_pressed = False
            if self.state in (ENGAGED, HOMING):
                self.state = IDLE
                self.get_logger().info('[B] Emergency stop → IDLE')

        # ── Homing runs independently of the Quest (no headset needed) ────
        if self.state == HOMING:
            self._step_homing()
            tcp_pos, tcp_rot = self._fk()
            self._publish_markers(tcp_pos, tcp_rot, None, None)
            self._publish_outputs()
            return

        if self.last_quest_msg is None:
            self._publish_markers(tcp_pos, tcp_rot, None, None)
            self._publish_outputs()
            return

        # Unpack Quest2 pose
        msg   = self.last_quest_msg
        q_pos = np.array([msg.pose.position.x,
                          msg.pose.position.y,
                          msg.pose.position.z])
        q_quat = [msg.pose.orientation.x, msg.pose.orientation.y,
                  msg.pose.orientation.z, msg.pose.orientation.w]
        q_rot  = _quat_to_mat(q_quat)

        # Handle A (toggle engage/disengage)
        if self.a_pressed:
            self.a_pressed = False
            if self.state == IDLE:
                self.state            = ENGAGED
                self.quest_anchor     = q_pos.copy()
                self.tcp_anchor       = tcp_pos.copy()
                self.rot_anchor       = tcp_rot.copy()
                self.quest_rot_anchor = q_rot.copy()
                self.fail_count       = 0
                self.cmd_filtered     = None   # re-seed smoothing on engage
                self.get_logger().info(
                    f'[A] ENGAGED — anchor TCP: {np.round(tcp_pos, 3)}')
            else:
                self.state = IDLE
                self.get_logger().info('[A] IDLE')

        # ── IK when ENGAGED ──────────────────────────────────────────────
        if self.state == ENGAGED:
            delta_q    = q_pos - self.quest_anchor
            target_pos = self.tcp_anchor + self.R_pos @ delta_q * self.scale

            target_rot = self._map_rotation(q_rot)

            q_seed = [0.0] * len(self.chain.links)
            for idx, nm in zip(self.active_idx, self.joint_names):
                q_seed[idx] = self.current_joints[nm]

            try:
                ik = self.chain.inverse_kinematics(
                    target_position=target_pos,
                    target_orientation=target_rot,
                    orientation_mode='all',
                    initial_position=q_seed)

                # Sanity guard: a huge single-tick raw jump = IK/singularity
                # flip, not real motion. Hold this tick rather than chase it.
                raw_max = max(
                    abs(float(ik[idx]) - self.current_joints[nm])
                    for idx, nm in zip(self.active_idx, self.joint_names))

                if raw_max > self.max_ik_jump:
                    self.get_logger().warning(
                        f'[IK-JUMP] {raw_max:.2f} rad > {self.max_ik_jump:.2f} '
                        '(likely singularity) — holding')
                    self.fail_count += 1
                else:
                    # Rate-limit: step each joint at most max_delta toward the
                    # target. Fast hand moves make the robot TRAIL smoothly and
                    # catch up — no violating step, no disengage.
                    self.fail_count = 0
                    steps = {nm: float(ik[idx]) - self.current_joints[nm]
                             for idx, nm in zip(self.active_idx, self.joint_names)}
                    if self.sync_rl:
                        # synchronized: scale ALL joints by one factor so the most-
                        # constrained hits max_delta and the rest scale with it →
                        # joints move/stop together (no staggered per-joint snaps,
                        # which cause control-box current transients / clicking).
                        max_step = max(abs(s) for s in steps.values())
                        if max_step > self.max_delta:
                            f = self.max_delta / max_step
                            steps = {nm: s * f for nm, s in steps.items()}
                    else:
                        # legacy: clip each joint independently (staggered stops)
                        steps = {nm: float(np.clip(s, -self.max_delta, self.max_delta))
                                 for nm, s in steps.items()}
                    for nm, s in steps.items():
                        self.current_joints[nm] += s

            except Exception as e:
                self.get_logger().warning(f'[IK] {e}')
                self.fail_count += 1

            if self.fail_count >= self.fail_max:
                self.state = IDLE
                self.get_logger().warn(
                    f'[DISENGAGED] {self.fail_count} IK/singularity failures → IDLE')

        # Log state transitions
        if self.state != prev_state:
            self.get_logger().info(f'{_NAME[prev_state]} → {_NAME[self.state]}')

        # Compute hand display position
        if self.state == ENGAGED and self.quest_anchor is not None:
            delta_q       = q_pos - self.quest_anchor
            hand_in_robot = self.tcp_anchor + self.R_pos @ delta_q * self.scale
            # Mapped hand orientation
            hand_rot   = self._map_rotation(q_rot)
        else:
            hand_in_robot = tcp_pos.copy()   # hand sphere sits on TCP when idle
            hand_rot      = tcp_rot.copy()

        self._publish_markers(tcp_pos, tcp_rot, hand_in_robot, hand_rot)
        self._publish_outputs()

    # ── RViz markers ─────────────────────────────────────────────────────
    def _publish_markers(self, tcp_pos, tcp_rot, hand_pos, hand_rot):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        _id = [0]

        def next_id():
            i = _id[0]; _id[0] += 1; return i

        def _base(id_):
            m = Marker()
            m.header.frame_id    = 'base_link'
            m.header.stamp       = now
            m.ns                 = 'teleop'
            m.id                 = id_
            m.action             = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        # ── Status indicator: colored text above robot (no sphere) ─────
        txt = _base(next_id())
        txt.type = Marker.TEXT_VIEW_FACING
        txt.pose.position.x = 0.0
        txt.pose.position.y = 0.0
        txt.pose.position.z = 1.3          # above robot base
        txt.scale.z = 0.07                 # text height
        if self.state == ENGAGED:
            txt.color.r, txt.color.g, txt.color.b, txt.color.a = 0.1, 0.9, 0.1, 1.0
            txt.text = 'TELEOP ON  [A=off  B=stop]'
        elif self.state == HOMING:
            txt.color.r, txt.color.g, txt.color.b, txt.color.a = 1.0, 0.6, 0.0, 1.0
            txt.text = 'HOMING...'
        else:
            txt.color.r, txt.color.g, txt.color.b, txt.color.a = 0.9, 0.1, 0.1, 1.0
            txt.text = 'TELEOP OFF  [A=on]'
        arr.markers.append(txt)

        # ── TCP sphere ───────────────────────────────────────────────
        tcp_s = _base(next_id())
        tcp_s.type = Marker.SPHERE
        tcp_s.pose.position.x = float(tcp_pos[0])
        tcp_s.pose.position.y = float(tcp_pos[1])
        tcp_s.pose.position.z = float(tcp_pos[2])
        tcp_s.scale.x = tcp_s.scale.y = tcp_s.scale.z = 0.04
        tcp_s.color.r, tcp_s.color.g, tcp_s.color.b, tcp_s.color.a = 1.0, 0.4, 0.0, 0.95
        arr.markers.append(tcp_s)

        # ── Axes at TCP ──────────────────────────────────────────────
        if tcp_rot is not None:
            self._axes_markers(tcp_pos, tcp_rot, 0.12, arr, next_id)

        # ── Quest2 hand sphere ───────────────────────────────────────
        if hand_pos is not None:
            hs = _base(next_id())
            hs.type = Marker.SPHERE
            hs.pose.position.x = float(hand_pos[0])
            hs.pose.position.y = float(hand_pos[1])
            hs.pose.position.z = float(hand_pos[2])
            hs.scale.x = hs.scale.y = hs.scale.z = 0.035
            hs.color.r, hs.color.g, hs.color.b, hs.color.a = 0.6, 0.0, 0.9, 0.9
            arr.markers.append(hs)

            # Axes at Quest2 hand
            if hand_rot is not None:
                self._axes_markers(hand_pos, hand_rot, 0.10, arr, next_id)

        self.marker_pub.publish(arr)

    def _axes_markers(self, pos, rot, length, arr, next_id):
        """Publish XYZ axes arrows. X=red, Y=green, Z=blue."""
        colors = [(1.0, 0.1, 0.1), (0.1, 1.0, 0.1), (0.1, 0.3, 1.0)]
        for i, (r, g, b) in enumerate(colors):
            m = Marker()
            m.header.frame_id    = 'base_link'
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = 'teleop'
            m.id                 = next_id()
            m.action             = Marker.ADD
            m.type               = Marker.ARROW
            m.pose.orientation.w = 1.0
            m.scale.x = 0.008   # shaft diameter
            m.scale.y = 0.015   # head diameter
            m.scale.z = 0.0
            axis = rot[:, i] * length
            p0 = Point(x=float(pos[0]),          y=float(pos[1]),          z=float(pos[2]))
            p1 = Point(x=float(pos[0]+axis[0]),  y=float(pos[1]+axis[1]),  z=float(pos[2]+axis[2]))
            m.points = [p0, p1]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            arr.markers.append(m)

    # ── Output publishing ─────────────────────────────────────────────────
    def _publish_outputs(self):
        if self.sim_mode:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name         = self.joint_names
            msg.position     = [self.current_joints[n] for n in self.joint_names]
            self.js_pub.publish(msg)
        elif self.state in (ENGAGED, HOMING):
            raw = [self.current_joints[n] for n in self.joint_names]
            # EMA low-pass on the teleop command to kill tremor/tracking jitter.
            # Skip during HOMING (its ramp is already smooth) and when alpha>=1.
            if self.state == ENGAGED and self.smooth_a < 1.0:
                a = self.smooth_a
                if self.cmd_filtered is None:
                    self.cmd_filtered = list(raw)       # seed = current pose, no jump
                else:
                    self.cmd_filtered = [a * r + (1.0 - a) * f
                                         for r, f in zip(raw, self.cmd_filtered)]
                out = self.cmd_filtered
            else:
                self.cmd_filtered = None                 # reset so re-engage re-seeds
                out = raw
            cmd = Float64MultiArray()
            cmd.data = out
            self.cmd_pub.publish(cmd)

    def __del__(self):
        if hasattr(self, '_urdf_path') and os.path.exists(self._urdf_path):
            os.remove(self._urdf_path)


def main(args=None):
    rclpy.init(args=args)
    node = UR10eProximityTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
