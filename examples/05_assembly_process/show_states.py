import json
import os

from compas.data import DataDecoder, json_dumps
from compas_fab.planning import AssemblyProcess

# Load Process File from JSON

HERE = os.path.dirname(__file__)

with open(os.path.join(HERE, 'process.json'), 'r') as f:
    assembly_process = json.load(f, cls=DataDecoder) #type: AssemblyProcess

# Print Initial State
initial_state = assembly_process.get_initial_state()
print(json_dumps(initial_state.to_data(), pretty=True))

# Print last state
last_state = assembly_process.get_intermediate_state(len(assembly_process.actions), debug=True)
print(json_dumps(last_state.to_data(), pretty=True))

