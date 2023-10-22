import json
import os, logging
from tracemalloc import start
from termcolor import colored

from compas.data import DataDecoder, DataEncoder
from compas_fab.planning import AssemblyProcess, SceneState
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint, Duration, CollisionMesh
from compas_fab_pychoreo.conversions import pose_from_frame

import pybullet_planning as pp
from state import initialize_process_scene_state, load_robot, set_state
from acm import add_acm, remove_acm
from motions import get_ik_generator
from utils import LOGGER

HERE = os.path.dirname(__file__)

# Load Process File from JSON
with open(os.path.join(HERE, 'process.json'), 'r') as f:
    assembly_process = json.load(f, cls=DataDecoder) #type: AssemblyProcess

viewer = False
debug = True
write = True
max_ik_attempts = 2000

options = {
    'debug': debug,
    'diagnosis': False
    }

logging_level = logging.DEBUG if debug else logging.INFO
LOGGER.setLevel(logging_level)

# Initialize PyChoreoClient
with PyChoreoClient(viewer=viewer) as client:
    initialize_process_scene_state(client, assembly_process)
    robot = load_robot(client, 'abb_crb15000')

    # Add Static Collision Meshes
    for id in assembly_process.static_collision_meshes:
        mesh = assembly_process.static_collision_meshes[id]
        cm = CollisionMesh(mesh, id)
        client.add_collision_mesh(cm)

    # Set Initial State
    initial_state = assembly_process.get_initial_state()
    set_state(client, robot, initial_state)

    ik_generator = get_ik_generator(client, robot, max_attempts=max_ik_attempts,  options=options)

    for action in assembly_process.get_robotic_actions():
        action_index = assembly_process.actions.index(action)
        start_state = assembly_process.get_intermediate_state(action_index)
        end_state = assembly_process.get_intermediate_state(action_index+1)

        acm_name = '_action_acm'
        add_acm(client, start_state, 'acm_name', action.allowed_collision_pairs)

        start_conf = assembly_process.get_action_starting_configuration(action)
        start_frame = start_state.robot_state.frame

        end_conf = assembly_process.get_action_ending_configuration(action)
        end_frame = action.robot_target

        set_state(client, robot, start_state, options)

        if debug and viewer:
            pp.draw_pose(pose_from_frame(start_frame), length=0.1)
            pp.add_text('Start', start_frame.point, color=(1,0,0))
            pp.draw_pose(pose_from_frame(end_frame), length=0.1)
            pp.add_text('End', end_frame.point, color=(1,0,0))

        if start_conf is not None:
            start_conf_in_collision = client.check_collisions(robot, start_conf, options=options)
            if start_conf_in_collision:
                LOGGER.warning(f"Start configuration of action {action_index} ({action.__class__.__name__}) is in collision.")
        else:
            if start_frame is not None:
                for start_conf in ik_generator(start_frame):
                    if debug:
                        client.set_robot_configuration(robot, start_conf)
                        pp.wait_if_gui('Start conf found.')
                    break
                else:
                    LOGGER.warning(f"Start state of action {action_index} ({action.__class__.__name__}) does not have a valid IK solution.")

        set_state(client, robot, end_state, options)
        if end_conf is not None:
            end_conf_in_collision = client.check_collisions(robot, end_conf, options=options)
            if end_conf_in_collision:
                LOGGER.warning(f"End configuration of action {action_index} ({action.__class__.__name__}) is in collision.")
        else:
            if end_frame is not None:
                for end_conf in ik_generator(end_frame):
                    # * save back to the process
                    if debug:
                        client.set_robot_configuration(robot, end_conf)
                        pp.wait_if_gui('End conf found.')
                    break
                else:
                    LOGGER.warning(f"End state of action {action_index} ({action.__class__.__name__}) does not have a valid IK solution.")

        if start_conf is not None and end_conf is not None:
            # save the found IK solutions back to the process
            action.planned_trajectory = JointTrajectory(trajectory_points=[start_conf, end_conf], joint_names=start_conf.joint_names,
                start_configuration=start_conf, fraction=1.0)

        remove_acm(client, acm_name)

if write:
    with open(os.path.join(HERE, 'process.json'), 'w') as f:
        json.dump(assembly_process, f, cls=DataEncoder, indent=4, sort_keys=True)
    LOGGER.info(colored('Process saved to process.json', 'green'))

LOGGER.info('Check IK finished!')