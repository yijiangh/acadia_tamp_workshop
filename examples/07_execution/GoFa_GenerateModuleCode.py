# This file will load the AssemblyProcess JSON file with planned trajectory
# and convert the planned trajectory into a module code for the GoFa robot.

# The module code will be saved in a file called "GoFa_GeneratedModule.modx".

# The module code can be loaded into the GoFa robot using the "Load Module"
# under the "Code" tab in the GoFa Teach Pendant.

# Load Process JSON file
import json
import math
import os

from compas.data import DataDecoder, json_dumps
from compas_fab.planning import AssemblyProcess, FreeMovement, LinearMovement, RoboticMovement

# Load Process File from JSON

HERE = os.path.dirname(__file__)

def load_process(file_name='process.json'):
    #type: (str) -> AssemblyProcess
    """Load an assembly process from a JSON file."""
    with open(os.path.join(HERE, file_name), 'r') as f:
        return json.load(f, cls=DataDecoder)

# This script loads a process from a JSON file and prints the number of actions, robotic actions and movement groups.
# This file also prints out whether the actions have planned trajectory or not.

# Load Process File from JSON

process = load_process()

actions = process.actions
print("Number of actions: {}".format(len(actions)))

# # Check that all robotic actions are planned
# for action in process.get_robotic_actions():
#     action_index = process.actions.index(action)
#     if action.planned_trajectory is None:
#         raise Exception("Action index={} is a robotic action but has no planned trajectory.".format(action_index))

from compas_fab.planning import RoboticMovement, LinearMovement, FreeMovement, OpenGripper, CloseGripper, LoadWorkpiece
programme_code = ""
for action in process.actions:
    action_index = process.actions.index(action)
    action_type_string = action.__class__.__name__
    programme_code += '  TPWrite "Action {} {}.";\n'.format(action_index, action_type_string)

    if isinstance(action, RoboticMovement):
        # Generate code from trajectory, the last point in the trajector uses a fine point.
        if action.planned_trajectory is None:
            # Skipping this action, since it has no planned trajectory
            continue
        configurations = action.planned_trajectory.points
        for i, conf in enumerate(configurations):
            values = list(conf.joint_values)
            values = [math.degrees(v) for v in values]
            target_string = "[[{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}],[9E9,9E9,9E9,9E9,9E9,9E9]]".format(values[0], values[1], values[2], values[3], values[4], values[5])
            if i < len(configurations) - 1:
                command_string = "  MoveAbsJ " + target_string + ",speeddata0,DefaultBlend,tool0;"
            else:
                command_string = "  MoveAbsJ " + target_string + ",speeddata0,FinePoint,tool0;"
            programme_code += command_string + "\n"

    if isinstance(action, OpenGripper):
        programme_code += '  TPWrite "Press Play to Continue";\n'
        programme_code += '  Stop;\n'
    if isinstance(action, CloseGripper):
        programme_code += '  TPWrite "Press Play to Continue";\n'
        programme_code += '  Stop;\n'
    if isinstance(action, LoadWorkpiece):
        programme_code += '  TPWrite "Press Play to Continue";\n'
        programme_code += '  Stop;\n'

module_code = ""
# Generate Module Code, part 1, copy header from GoFa_Module_Start.txt
with open(os.path.join(HERE,"GoFa_Module_Start.txt"), 'r') as f:
    module_code += f.read()

# Generate Module Code, part 2, copy actions from process
module_code += programme_code

# Generate Module Code, part 3, copy footer from GoFa_Module_End.txt
with open(os.path.join(HERE,"GoFa_Module_End.txt"), 'r') as f:
    module_code += f.read()

# Save Module Code to file (overwrite if file already exists)
with open(os.path.join(HERE,"GoFa_GeneratedModule.modx"), 'w') as f:
    f.write(module_code)

