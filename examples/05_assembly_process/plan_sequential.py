import os
import json
import logging
import pybullet_planning as pp
from termcolor import colored

from compas.data import DataDecoder, DataEncoder

from compas_fab.planning.action import FreeMovement, LinearMovement, RoboticMovement
from compas_fab.planning import AssemblyProcess

from compas_fab.robots import CollisionMesh

from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.conversions import pose_from_frame

from state import initialize_process_scene_state, load_robot, set_state
from motions import plan_free_movement, plan_linear_movement
from utils import LOGGER, get_tolerances

############################

HERE = os.path.dirname(__file__)
with open(os.path.join(HERE, 'process.json'), 'r') as f:
    process = json.load(f, cls=DataDecoder) #type: AssemblyProcess

viewer = False
write = True
debug = False
watch_traj = False
diagnosis = False

logging_level = logging.DEBUG if debug else logging.INFO
LOGGER.setLevel(logging_level)

options = {
    'debug': debug,
    'diagnosis': diagnosis,
    'rrt_restarts': 20,
    'max_ik_attempts': 500,
    }

# Remove all the trajectories from the process
for action in process.get_robotic_actions():
    action.planned_trajectory = None

with PyChoreoClient(viewer=viewer) as client:
    initialize_process_scene_state(client, process)
    robot = load_robot(client, 'abb_crb15000')
    options.update(get_tolerances(robot, super_res=True))

    set_state(client, robot, process.get_initial_state(), options)
    # pp.wait_if_gui('Initial State')

    for action in process.get_robotic_actions():
        if not isinstance(action, RoboticMovement):
            continue

        trajectory = None
        action_index = process.actions.index(action)
        state = process.get_intermediate_state(action_index)

        start_conf = process.get_action_starting_configuration(action)
        start_frame = state.robot_state.frame

        end_conf = process.get_action_ending_configuration(action)
        end_frame = action.robot_target

        if debug and viewer:
            pp.draw_pose(pose_from_frame(start_frame), length=0.1)
            pp.add_text('Start', start_frame.point, color=(1,0,0))
            pp.draw_pose(pose_from_frame(end_frame), length=0.1)
            pp.add_text('End', end_frame.point, color=(1,0,0))

        if isinstance(action, FreeMovement):
            LOGGER.debug(colored('Planning Free Movement {}'.format(action_index), 'cyan'))
            trajectory = plan_free_movement(client, robot,
                                            state, action.allowed_collision_pairs,
                                            start_conf, end_conf,
                                            start_frame, end_frame,
                                            group=None, options=options)
        elif isinstance(action, LinearMovement):
            LOGGER.debug(colored('Planning Linear Movement {}'.format(action_index), 'cyan'))
            trajectory = plan_linear_movement(client, robot,
                                              state, action.allowed_collision_pairs,
                                              start_conf, end_conf,
                                              start_frame, end_frame,
                                              max_ik_attempts=options['max_ik_attempts'],
                                              group=None, options=options)

        if trajectory is not None:
            LOGGER.info(colored('Trajectory found for movement {}: {} points'.format(action_index, len(trajectory.points)), 'green'))
            action.planned_trajectory = trajectory

            # replay for debugging
            if watch_traj and viewer:
                set_state(client, robot, state, options)
                for conf in trajectory.points:
                    client.set_robot_configuration(robot, conf)
                    pp.wait_if_gui('Step traj')
        else:
            LOGGER.info(colored('Trajectory NOT found for movement {}'.format(action_index), 'red'))
            # raise ValueError("Trajectory not found")

        if debug and viewer:
            pp.remove_all_debug()

if write:
    with open(os.path.join(HERE, 'process.json'), 'w') as f:
        json.dump(process, f, cls=DataEncoder, indent=4, sort_keys=True)
    LOGGER.info(colored('Process saved to process.json', 'green'))

