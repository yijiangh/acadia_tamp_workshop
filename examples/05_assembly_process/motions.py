import pybullet_planning as pp
from typing import List, Tuple

from compas.robots import RobotModel
from compas.geometry import Frame

from compas_fab.robots import Configuration, JointTrajectory, JointTrajectoryPoint, Duration
from compas_fab.planning import SceneState

from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.conversions import pose_from_frame

from state import set_state
from acm import add_acm, remove_acm
from utils import LOGGER

IKFAST_INSTALLED = False
try:
    import ikfast_abb_crb15000_5_95
except ImportError:
    IKFAST_INSTALLED = False

def get_ik_generator(client: PyChoreoClient, robot: RobotModel, group=None, max_attempts=50, options=None):
    options = options or {}

    robot_uid = client.get_robot_pybullet_uid(robot)
    ik_joint_names = robot.get_configurable_joint_names(group=group)
    ik_joints = pp.joints_from_names(robot_uid, ik_joint_names)
    joint_types = robot.get_joint_types_by_names(ik_joint_names)
    sample_fn = pp.get_sample_fn(robot_uid, ik_joints)

    def ik_generator(frame_WCF: Frame, initial_guess_conf: Configuration=None):
        for iteration in range(max_attempts+1):
            # use the provided initial guess before trying random samples
            if iteration == 0 and initial_guess_conf is not None:
                start_conf = initial_guess_conf
            else:
                start_conf = Configuration(joint_values=sample_fn(), joint_types=joint_types, joint_names=ik_joint_names)

            client.set_robot_configuration(robot, start_conf)

            configuration = client.inverse_kinematics(robot, frame_WCF, start_configuration=start_conf, group=group, options=options)
            if configuration is not None:
                LOGGER.debug('IK found in {} iterations'.format(iteration))
                yield configuration
        else:
            if IKFAST_INSTALLED:
                target_pose = pose_from_frame(frame_WCF)
                pos = pp.point_from_pose(target_pose)
                rot = pp.matrix_from_quat(pp.quat_from_pose(target_pose)).tolist()
                conf_candidates = ikfast_abb_crb15000_5_95.get_ik(pos, rot, [])
                for joint_values in conf_candidates:
                    conf = Configuration(joint_values=joint_values, joint_types=joint_types, joint_names=ik_joint_names)
                    if not client.check_collisions(robot, conf, options=options):
                        LOGGER.debug('IK found by Analytical IKFast')
                        yield conf

            LOGGER.debug('IK not found in {} iterations'.format(max_attempts))

    return ik_generator

def plan_free_movement(client: PyChoreoClient, 
                       robot: RobotModel, 
                       state: SceneState, 
                       allowed_collision_pairs: List[Tuple[str, str]]=None, 
                       start_conf=None, end_conf=None, 
                       start_frame=None, end_frame=None,
                       group=None,
                       max_ik_attempts=100,
                       options=None):
    options = options or {}
    allowed_collision_pairs = allowed_collision_pairs or []

    set_state(client, robot, state, options)
    initial_guess_conf = client.get_robot_configuration(robot, group=group)

    if start_conf is not None:
        def _start_ik_generator(frame, init_conf):
            yield start_conf
        start_ik_generator = _start_ik_generator
    else:
        start_ik_generator = get_ik_generator(client, robot, group, max_ik_attempts)
        assert start_frame is not None

    if end_conf is not None:
        def _end_ik_generator(frame, init_conf):
            yield end_conf
        end_ik_generator = _end_ik_generator
    else:
        end_ik_generator = get_ik_generator(client, robot, group, max_ik_attempts)
        assert end_frame is not None

    acm_name = '_fmp_acm'
    add_acm(client, state, acm_name, allowed_collision_pairs)

    trajectory = None
    for start_conf in start_ik_generator(start_frame, initial_guess_conf):
        for end_conf in end_ik_generator(end_frame, start_conf):
            goal_constraint = robot.constraints_from_configuration(end_conf, [0.01], [0.01], group=group)
            trajectory = client.plan_motion(robot, goal_constraint, start_configuration=start_conf, group=group, options=options)

            if trajectory is not None:
                break
        if trajectory is not None:
            break

    remove_acm(client, acm_name)

    return trajectory
    
        
def plan_linear_movement(client, 
                       robot, 
                       state: SceneState, 
                       allowed_collision_pairs: List[Tuple[str, str]]=None, 
                       start_conf=None, end_conf=None, 
                       start_frame=None, end_frame=None,
                       group=None,
                       max_ik_attempts=100,
                       options=None):
    if start_conf is not None and end_conf is not None:
        # raise ValueError("Cannot specify both start and end configuration in a linear movement. Problem could be overly constrained. Please specify only one of them.")
        LOGGER.warning("Cannot specify both start and end configuration in a linear movement. Problem could be overly constrained. We will ignore the end configuration and plan forward.")
        end_conf = None

    options = options or {}
    allowed_collision_pairs = allowed_collision_pairs or []

    set_state(client, robot, state, options)
    initial_guess_conf = client.get_robot_configuration(robot, group=group)

    plan_forward = True
    if start_conf is not None:
        def _start_ik_generator(frame, init_conf):
            yield start_conf
        initial_ik_generator = _start_ik_generator
    else:
        initial_ik_generator = get_ik_generator(client, robot, group, max_ik_attempts)
        assert start_frame is not None

    if end_conf is not None:
        plan_forward = False
        def _end_ik_generator(frame, init_conf):
            yield end_conf
        initial_ik_generator = _end_ik_generator

    if plan_forward:
        target_frames = [start_frame, end_frame]
    else:
        target_frames = [end_frame, start_frame]

    acm_name = '_lmp_acm'
    add_acm(client, state, acm_name, allowed_collision_pairs)

    trajectory = None
    for initial_conf in initial_ik_generator(target_frames[0], initial_guess_conf):
        trajectory = client.plan_cartesian_motion(robot, target_frames, start_configuration=initial_conf,
            group=group, options=options)
        if trajectory is not None:
            break

    # reverse the trajectory
    if not plan_forward and trajectory is not None:
        jt_traj_pts = []
        joint_names = trajectory.points[0].joint_names
        for i, conf in enumerate(trajectory.points[::-1]):
            jt_traj_pt = JointTrajectoryPoint(conf.joint_values, conf.joint_types, time_from_start=Duration(i*1,0))
            jt_traj_pt.joint_names = conf.joint_names
            jt_traj_pts.append(jt_traj_pt)
        trajectory = JointTrajectory(trajectory_points=jt_traj_pts, joint_names=joint_names,
            start_configuration=jt_traj_pts[0], fraction=1.0)

    remove_acm(client, acm_name)

    return trajectory