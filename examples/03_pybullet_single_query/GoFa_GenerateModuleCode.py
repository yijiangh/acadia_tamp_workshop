# This file will load the AssemblyProcess JSON file with planned trajectory
# and convert the planned trajectory into a module code for the GoFa robot.

# The module code will be saved in a file called "GoFa_GeneratedModule.modx".

# The module code can be loaded into the GoFa robot using the "Load Module"
# under the "Code" tab in the GoFa Teach Pendant.

# Load Process JSON file
import json
import os
import math

from compas.data import DataDecoder, json_dumps
from compas_fab.planning import AssemblyProcess, FreeMovement, LinearMovement, RoboticMovement

# Load Process File from JSON

HERE = os.path.dirname(__file__)

def load(file_name='process.json'):
    #type: (str) -> AssemblyProcess
    """Load an assembly process from a JSON file."""
    with open(os.path.join(HERE, file_name), 'r') as f:
        return json.load(f, cls=DataDecoder)

# This script loads a process from a JSON file and prints the number of actions, robotic actions and movement groups.
# This file also prints out whether the actions have planned trajectory or not.

# Load Process File from JSON

trajectory = load('free_motion_result.json')

configurations = trajectory.points
programme_code = ""

for i, conf in enumerate(configurations):
    values = list(conf.joint_values)
    values = [math.degrees(v) for v in values]
    target_string = "[[{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}],[9E9,9E9,9E9,9E9,9E9,9E9]]".format(values[0], values[1], values[2], values[3], values[4], values[5])
    if i < len(configurations) - 1:
        command_string = "  MoveAbsJ " + target_string + ",speeddata0,DefaultBlend,tool0;"
    else:
        command_string = "  MoveAbsJ " + target_string + ",speeddata0,FinePoint,tool0;"
    programme_code += command_string + "\n"

configurations = configurations[::-1]
for i, conf in enumerate(configurations):
    values = list(conf.joint_values)
    values = [math.degrees(v) for v in values]
    target_string = "[[{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}],[9E9,9E9,9E9,9E9,9E9,9E9]]".format(values[0], values[1], values[2], values[3], values[4], values[5])
    if i < len(configurations) - 1:
        command_string = "  MoveAbsJ " + target_string + ",speeddata0,DefaultBlend,tool0;"
    else:
        command_string = "  MoveAbsJ " + target_string + ",speeddata0,FinePoint,tool0;"
    programme_code += command_string + "\n"

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

