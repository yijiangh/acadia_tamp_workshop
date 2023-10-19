import json
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