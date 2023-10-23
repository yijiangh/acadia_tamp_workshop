import json
import os, logging

from compas.data import DataDecoder

from compas_fab.planning import AssemblyProcess, SceneState
from compas_fab_pychoreo.client import PyChoreoClient
from compas.robots import RobotModel

import pybullet_planning as pp
from state import initialize_process_scene_state, load_robot, set_state
from acm import add_acm, remove_acm
from utils import LOGGER
HERE = os.path.dirname(__file__)

def check_state_collisions(client: PyChoreoClient, robot : RobotModel, state: SceneState, options=None):
    options = options or {}
    debug = options.get('debug', False)

    # * update state
    set_state(client, robot, state, options)

    # * check collisions among the list of attached objects and obstacles in the scene.
    # This includes collisions between:
    #     - each pair of (attached object, obstacle)
    #     - each pair of (attached object 1, attached object 2)
    in_collision = client.check_attachment_collisions(options)

    # * check collision with the robot if a robot configuration is stored in the scene state
    if state.robot_state.configuration:
        in_collision = in_collision or client.check_collisions(robot, state.robot_state.configuration, options=options)

    if debug and in_collision:
        client._print_object_summary()

    return in_collision

# Load Process File from JSON
with open(os.path.join(HERE, 'process.json'), 'r') as f:
    assembly_process = json.load(f, cls=DataDecoder) #type: AssemblyProcess

viewer = True
debug = True
options = {
    'debug': debug,
    'diagnosis': True
    }

logging_level = logging.DEBUG if debug else logging.INFO
LOGGER.setLevel(logging_level)

# Initialize PyChoreoClient
with PyChoreoClient(viewer=viewer) as client:
    initialize_process_scene_state(client, assembly_process)
    robot = load_robot(client, 'abb_crb15000')

    # Set Initial State
    initial_state = assembly_process.get_initial_state()
    set_state(client, robot, initial_state)

    failed_action_ids = []
    for action in assembly_process.get_robotic_actions():
        action_index = assembly_process.actions.index(action)
        action = assembly_process.actions[action_index]

        start_state = assembly_process.get_intermediate_state(action_index)

        acm_name = '_action_acm'
        add_acm(client, start_state, 'acm_name', action.allowed_collision_pairs)

        start_state_in_collision = check_state_collisions(client, robot, start_state, options)
        if start_state_in_collision:
            LOGGER.warning(f"Start state of action {action_index} ({action.__class__.__name__}) is in collision.")

        end_state = assembly_process.get_intermediate_state(action_index+1)
        end_state_in_collision = check_state_collisions(client, robot, end_state, options)
        if end_state_in_collision:
            LOGGER.warning(f"End state of action {action_index} ({action.__class__.__name__}) is in collision.")

        remove_acm(client, acm_name)

        if start_state_in_collision or end_state_in_collision:
            failed_action_ids.append(action_index)

LOGGER.info('Check state finished!')
if len(failed_action_ids) > 0:
    LOGGER.warning(f'Actions that needs care and love (in collision): {failed_action_ids}')