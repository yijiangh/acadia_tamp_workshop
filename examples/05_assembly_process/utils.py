import logging
import math
from compas.robots import RobotModel
from compas.robots import Joint

def get_logger(name):
    logger = logging.getLogger(name)

    try:
        from colorlog import ColoredFormatter
        formatter = ColoredFormatter("%(log_color)s%(levelname)-8s%(reset)s %(white)s%(message)s",
                                     datefmt=None,
                                     reset=True,
                                     log_colors={'DEBUG': 'cyan', 'INFO': 'green',
                                                 'WARNING': 'yellow',
                                                 'ERROR': 'red', 'CRITICAL': 'red',
                                                 }
                                     )
    except ImportError:
        formatter = logging.Formatter('%(asctime)s | %(name)s | %(levelname)s | %(message)s')

    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)

    return logger

LOGGER = get_logger('robarch_pddl')

#######################

def get_tolerances(robot: RobotModel, group=None, super_res=False):
    joint_names = robot.get_configurable_joint_names(group)
    joint_types = robot.get_joint_types_by_names(joint_names)
    res_ratio = 0.5 if super_res else 1.0
    # TODO joint resolution and weight from joint name
    # * threshold to check joint flipping
    joint_jump_tolerances = {}
    joint_compare_tolerances = {}
    joint_resolutions = {}
    for jt_name, jt_type in zip(joint_names, joint_types):
        # 0.1 rad = 5.7 deg
        if jt_type == Joint.REVOLUTE:
            joint_jump_tolerances[jt_name] = 10.0 * math.pi / 180.0 # 0.174 rad
            joint_resolutions[jt_name] = 2 * math.pi / 180.0 * res_ratio# 0.174 rad
            joint_compare_tolerances[jt_name] = 0.0017 # rad, try tightened to 0.001 if possible
        elif jt_type == Joint.PRISMATIC:
            joint_jump_tolerances[jt_name] = 0.05 # meter
            joint_resolutions[jt_name] = 0.05 * res_ratio # meter
            joint_compare_tolerances[jt_name] = 1e-5
        else:
            raise ValueError("Strange joint type {} | {}".format(jt_type, jt_name))

    tolerances = {
        'joint_jump_tolerances' : joint_jump_tolerances,
        'joint_compare_tolerances' : joint_compare_tolerances,
        'frame_compare_distance_tolerance' : 0.0011, # meter
        'frame_compare_axis_angle_tolerance' : 0.0025, # rad
        'joint_resolutions' : joint_resolutions,
        # 'joint_custom_limits' : get_gantry_robot_custom_limits(MAIN_ROBOT_ID),
        # the collision is counted when penetration distance is bigger than this value
        'collision_distance_threshold' : 0.0012, # in meter,
    }
    return tolerances

