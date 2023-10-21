import os
import json
import logging
import pybullet_planning as pp
from termcolor import colored

from compas.data import DataDecoder, DataEncoder
from typing import List

from functions import load_process
from compas_fab.planning import AssemblyProcess, FreeMovement, LinearMovement, RoboticMovement

from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.conversions import pose_from_frame

from state import initialize_process_scene_state, load_robot, set_state
from motions import plan_free_movement, plan_linear_movement
from utils import LOGGER, get_tolerances

# The scope of the planning process includes the following options:
#
# ``SINGLE_MOVEMENT``: Plan a single movement (LinearMovement or FreeMovement).
# ``SINGLE_LMG``: Plan a single linear movement group.
# ``SINGLE_FMG``: Plan a single free movement group.
# ``ALL_LMG``: Plan all linear movement groups.
# ``ALL_FMG``: Plan all free movement groups.
# ``ALL``: Plan all movements.  First all LMG, then all FMG.

PLANNING_SCOPE = [
    'INDIVIDUAL_MOVEMENTS',
    'SINGLE_LMG',
    'SINGLE_FMG',
    'ALL_LMG',
    'ALL_FMG',
    'ALL',
]

planning_scope = 'ALL'

# For ``SINGLE_LMG`` and ``SINGLE_FMG`` scopes, specify the index of the movement group to plan.
# The index refers to the index among all actions in process, as in process.actions[single_group_action_index].
single_group_action_index = 39

# For ``INDIVIDUAL_MOVEMENTS`` scope, specify the indices of the movement to plan.
individual_movement_indices = [3,4,6,7]

# Parase the scope and single_group_action_index to determine which movements to plan.
# If these movements have already been planned, their old trajectory will be overwritten.
# The old trajectory will be removed even if the new planning fails.

process = load_process() # type: AssemblyProcess

staged_actions = []  # type: List[RoboticMovement]
staged_lmgs = []     # type: List[List[LinearMovement]]
staged_fmgs = []   # type: List[List[FreeMovement]]

if planning_scope == 'INDIVIDUAL_MOVEMENTS':
    for action_index in individual_movement_indices:
        staged_actions.append(process.actions[action_index])
if planning_scope == 'SINGLE_LMG':
    movement = process.actions[single_group_action_index]
    assert isinstance(movement, LinearMovement), "Movement {} (index = {}) must be a LinearMovement when using SINGLE_LMG scope".format(
        movement, single_group_action_index)
    staged_lmgs.append(process.get_linear_movement_group(movement))
if planning_scope == 'SINGLE_FMG':
    movement = process.actions[single_group_action_index]
    assert isinstance(movement, FreeMovement), "Movement {} (index = {}) must be a FreeMovement when using SINGLE_FMG scope".format(
        movement, single_group_action_index)
    staged_fmgs.append(process.get_free_movement_group(movement))
if planning_scope == 'ALL_LMG':
    staged_lmgs = process.get_linear_movement_groups()
if planning_scope == 'ALL_FMG':
    staged_fmgs = process.get_free_movement_groups()
if planning_scope == 'ALL':
    staged_actions = process.get_robotic_actions()
    staged_lmgs = process.get_linear_movement_groups()
    staged_fmgs = process.get_free_movement_groups()


assert all([isinstance(action, RoboticMovement) for action in staged_actions])
assert all([[isinstance(action, LinearMovement)
           for action in group] for group in staged_lmgs])
assert all([[isinstance(action, FreeMovement) for action in group]
           for group in staged_fmgs])

print("Planning scope: {}".format(planning_scope))
if planning_scope != 'INDIVIDUAL_MOVEMENTS':
    # Expand the list of lists to a list of actions
    staged_actions = [action for group in staged_lmgs +
                      staged_fmgs for action in group]
    print("Number of Linear Movement Groups to plan: {}".format(len(staged_lmgs)))
    print("Number of Free Movement Groups to plan: {}".format(len(staged_fmgs)))
print("Total Number of Actions to plan: {}".format(len(staged_actions)))


# Remove planned trajectory from the staged actions.
# This is necessary to avoid the planner constrained by configurations of old neighbour trajectory.

for action in staged_actions:
    action.planned_trajectory = None

###################
# Actual planning computation starts here
viewer = False
write = True
debug = False
diagnosis = False

logging_level = logging.DEBUG if debug else logging.INFO
LOGGER.setLevel(logging_level)

options = {
    'debug': debug,
    'diagnosis': diagnosis,
    'rrt_restarts': 20,
    'max_ik_attempts': 500,
    }

with PyChoreoClient(viewer=viewer) as client:
    initialize_process_scene_state(client, process)
    robot = load_robot(client, 'abb_crb15000')
    options.update(get_tolerances(robot, super_res=True))

    set_state(client, robot, process.get_initial_state(), options)

    # Plan the staged LMG
    # The planner will be constrained by the configurations of the neighbour trajectories.

    if planning_scope in ['SINGLE_LMG', 'ALL_LMG', 'ALL']:
        print("Planning Linear Movement Groups")
        for group in staged_lmgs:
            print("- Planning Linear Movement Group")
            success = False
            for action in group:
                action_index = process.actions.index(action)
                scene_state = process.get_intermediate_state(action_index)

                start_conf = process.get_action_starting_configuration(action)
                start_frame = scene_state.robot_state.frame

                end_conf = process.get_action_ending_configuration(action)
                end_frame = action.robot_target

                print("- - Planning (index={}) LinearMovement".format(action_index))
                trajectory = plan_linear_movement(client, robot,
                                                  scene_state, action.allowed_collision_pairs,
                                                  start_conf, end_conf,
                                                  start_frame, end_frame,
                                                  max_ik_attempts=options['max_ik_attempts'],
                                                  group=None, options=options)

                success = trajectory is not None
                if not success:
                    break
                action.planned_trajectory = trajectory

            # Clean the partially successful planning
            if not success:
                print("- - Failed to plan Linear Movement Group")
                for action in group:
                    action.planned_trajectory = None
        print ("Done Planning LMGs  \n\n")

    if planning_scope in ['SINGLE_FMG', 'ALL_FMG', 'ALL']:
        print("Planning Free Movement Groups")
        for group in staged_fmgs:
            print("- Planning Free Movement Group")
            for action in group:
                action_index = process.actions.index(action)
                scene_state = process.get_intermediate_state(action_index)

                start_conf = process.get_action_starting_configuration(action)
                start_frame = scene_state.robot_state.frame

                end_conf = process.get_action_ending_configuration(action)
                end_frame = action.robot_target

                print("- - Planning (index={}) FreeMovement".format(action_index))
                trajectory = plan_free_movement(client, robot,
                                                scene_state, action.allowed_collision_pairs,
                                                start_conf, end_conf,
                                                start_frame, end_frame,
                                                group=None, options=options)

                success = trajectory is not None
                if not success:
                    break
                action.planned_trajectory = trajectory

            # Clean the partially successful planning
            if not success:
                print("- - Failed to plan Linear Movement Group")
                for action in group:
                    action.planned_trajectory = None    
        print ("Done Planning FMGs  \n\n")

    if planning_scope in ['INDIVIDUAL_MOVEMENTS']:
        print("Planning Individual Movements")
        for action in staged_actions:
            action_index = process.actions.index(action)
            scene_state = process.get_intermediate_state(action_index)

            start_conf = process.get_action_starting_configuration(action)
            start_frame = scene_state.robot_state.frame

            end_conf = process.get_action_ending_configuration(action)
            end_frame = action.robot_target

            print("- - Planning (index={}) {}".format(action_index, action.__class__.__name__))
            if isinstance(action, FreeMovement):
                trajectory = plan_free_movement(client, robot,
                                                scene_state, action.allowed_collision_pairs,
                                                start_conf, end_conf,
                                                start_frame, end_frame,
                                                group=None, options=options)
            elif isinstance(action, LinearMovement):
                trajectory = plan_linear_movement(client, robot,
                                                  scene_state, action.allowed_collision_pairs,
                                                  start_conf, end_conf,
                                                  start_frame, end_frame,
                                                  max_ik_attempts=options['max_ik_attempts'],
                                                  group=None, options=options)

            success = trajectory is not None
            if not success:
                print("- - Failed to plan (index={}) {}".format(action_index, action.__class__.__name__))
                continue
            action.planned_trajectory = trajectory

        print ("Done Planning Individual Movements \n\n")

# Save the process to a JSON file.
# This will overwrite the original process file.
HERE = os.path.dirname(__file__)
if write:
    with open(os.path.join(HERE, 'process.json'), 'w') as f:
        json.dump(process, f, cls=DataEncoder, indent=4, sort_keys=True)
    LOGGER.info(colored('Process saved to process.json', 'green'))
