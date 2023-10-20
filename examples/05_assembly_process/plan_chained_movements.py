from functions import load_process
from compas_fab.planning import AssemblyProcess, FreeMovement, LinearMovement, RoboticMovement

from typing import List

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
single_group_action_index = 14

# For ``INDIVIDUAL_MOVEMENTS`` scope, specify the indices of the movement to plan.
individual_movement_indices = [3,4,6,7]

# Parase the scope and single_group_action_index to determine which movements to plan.
# If these movements have already been planned, their old trajectory will be overwritten.
# The old trajectory will be removed even if the new planning fails.

process = load_process()

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
            start_configuration = process.get_action_starting_configuration(action)
            end_configuration = process.get_action_ending_configuration(action)

            print("- - Planning (index={}) LinearMovement from {} to {}".format(action_index,
            # trajectory = action.plan(start_configuration, end_configuration, scene_state)
            # success = trajectory is not None
            # if not success:
            #     break
            # action.planned_trajectory = trajectory
                start_configuration, end_configuration))
        # Clean the partially successful planning
        # if not success:
        #     print("- - Failed to plan Linear Movement Group")
        #     for action in group:
        #         action.planned_trajectory = None
    print ("Done Planning LMGs  \n\n")

if planning_scope in ['SINGLE_FMG', 'ALL_FMG', 'ALL']:
    print("Planning Free Movement Groups")
    for group in staged_fmgs:
        print("- Planning Free Movement Group")
        for action in group:
            action_index = process.actions.index(action)
            scene_state = process.get_intermediate_state(action_index)
            start_configuration = process.get_action_starting_configuration(action)
            end_configuration = process.get_action_ending_configuration(action)
            # action.plan(start_configuration, end_configuration, scene_state)
            print("- - Planning (index={}) FreeMovement from {} to {}".format(action_index,
            # trajectory = action.plan(start_configuration, end_configuration, scene_state)
            # success = trajectory is not None
            # if not success:
            #     break
            # action.planned_trajectory = trajectory
                start_configuration, end_configuration))
        # Clean the partially successful planning
        # if not success:
        #     print("- - Failed to plan Linear Movement Group")
        #     for action in group:
        #         action.planned_trajectory = None    print ("Done Planning FMGs  \n\n")

if planning_scope in ['INDIVIDUAL_MOVEMENTS']:
    print("Planning Individual Movements")
    for action in staged_actions:
        action_index = process.actions.index(action)
        scene_state = process.get_intermediate_state(action_index)
        start_configuration = process.get_action_starting_configuration(action)
        end_configuration = process.get_action_ending_configuration(action)
        # action.plan(start_configuration, end_configuration, scene_state)
        print("- - Planning (index={}) {} from {} to {}".format(action_index,
            action.__class__.__name__, start_configuration, end_configuration))
    print ("Done Planning Individual Movements \n\n")


# Save the process to a JSON file.
# This will overwrite the original process file.
