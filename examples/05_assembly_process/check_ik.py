import json
import os

from compas.data import DataDecoder

from compas_fab.planning import AssemblyProcess, SceneState
from compas_fab_pychoreo.client import PyChoreoClient
from compas.robots import RobotModel

import pybullet_planning as pp
from state import initialize_process_scene_state, load_robot, set_state
from acm import add_acm
from motions import get_ik_generator
from utils import LOGGER

HERE = os.path.dirname(__file__)

# Load Process File from JSON
with open(os.path.join(HERE, 'process.json'), 'r') as f:
    assembly_process = json.load(f, cls=DataDecoder) #type: AssemblyProcess

viewer = False
max_ik_attempts = 100
options = {
    'debug': True,
    'diagnosis': False
    }

# Initialize PyChoreoClient
with PyChoreoClient(viewer=viewer) as client:
    initialize_process_scene_state(client, assembly_process)
    robot = load_robot(client, 'abb_crb15000')

    # Set Initial State
    initial_state = assembly_process.get_initial_state()
    set_state(client, robot, initial_state)

    ik_generator = get_ik_generator(client, robot, max_attempts=max_ik_attempts,  options=options)

    # Set Subsequent states
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
        if start_conf is not None:
            start_conf_in_collision = client.check_collisions(robot, start_conf, options=options)
            if start_conf_in_collision:
                LOGGER.warning(f"Start configuration of action {action_index} is in collision.")
        else:
            for start_conf in ik_generator(start_frame):
                if options['debug']:
                    client.set_robot_configuration(robot, start_conf)
                    pp.wait_if_gui('Start conf found.')
                break
            else:
                LOGGER.warning(f"Start state of action {action_index} does not have a valid IK solution.")

        set_state(client, robot, end_state, options)
        if end_conf is not None:
            end_conf_in_collision = client.check_collisions(robot, end_conf, options=options)
            if end_conf_in_collision:
                LOGGER.warning(f"End configuration of action {action_index} is in collision.")
        else:
            for end_conf in ik_generator(end_frame):
                if options['debug']:
                    client.set_robot_configuration(robot, end_conf)
                    pp.wait_if_gui('End conf found.')
                break
            else:
                LOGGER.warning(f"End state of action {action_index} does not have a valid IK solution.")

        # remove ACM of the current action
        if acm_name in client.extra_disabled_collision_links:
            del client.extra_disabled_collision_links[acm_name]

