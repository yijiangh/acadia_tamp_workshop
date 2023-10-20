import json
import os

from compas.data import DataDecoder, json_dumps
from compas_fab.planning import AssemblyProcess, LinearMovement, FreeMovement

from functions import load_process

# This script loads a process from a JSON file and prints the number of actions, robotic actions and movement groups.
# This file also prints out whether the actions have planned trajectory or not.

# Load Process File from JSON

process = load_process()

actions = process.actions
print("Number of actions: {}".format(len(actions)))

robotic_actions = process.get_robotic_actions()
print("Number of robotic actions: {}".format(len(robotic_actions)))

movement_groups = process.get_movement_groups()
print("Number of movement groups: {}".format(len(movement_groups)))

# Iterates over the actions groups.
for i, group in enumerate(movement_groups):
    if isinstance(group[0], LinearMovement):
        print("Group {} - Linear Movement Group".format(i))
    elif isinstance(group[0], FreeMovement):
        print("Group {} - Free Movement Group".format(i))

    # Iterates over the actions in the group.
    for action in group:
        action_index = process.actions.index(action)
        action_type_str = action.__class__.__name__
        has_trajectory = action.planned_trajectory is not None
        print("  Action {}: {} {}".format(action_index, action_type_str, "(Planned)" if has_trajectory else "(Not Planned)"))