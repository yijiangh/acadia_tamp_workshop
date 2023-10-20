import json
import os

from compas.data import DataDecoder, DataEncoder
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab.robots import CollisionMesh

HERE = os.path.dirname(__file__)
TOOL_EXPORT_DIR = os.path.join(HERE)
ROBOT_MODEL_DIR = os.path.join(HERE, '..', '..', 'robots')

with open(os.path.join(HERE, 'free_motion_request.json'), 'r') as f:
    request_data = json.load(f, cls=DataDecoder)

collision_meshes = request_data['collision_meshes']
start_conf = request_data['start_configuration']
end_conf = request_data['end_configuration']

with PyChoreoClient(viewer=False) as client:
    urdf_filename = os.path.join(ROBOT_MODEL_DIR, 'abb_crb15000_support', "urdf", "crb15000_5_95.urdf")
    srdf_filename = os.path.join(ROBOT_MODEL_DIR, 'abb_crb15000_support', "srdf", "crb15000_5_95.srdf")

    robot = client.load_robot(urdf_filename)
    client.load_semantics(robot, srdf_filename)

    for i, mesh in enumerate(collision_meshes):
        client.add_collision_mesh(CollisionMesh(mesh, f'collision_mesh_{i}'))

    options = {
        'diagnosis': True,
        'solve_timeout': 20.0, # This does not work yet
    }

    goal_constraint = robot.constraints_from_configuration(end_conf, [0.01], [0.01])
    trajectory = client.plan_motion(robot, goal_constraint, start_configuration=start_conf, options=options)

    if trajectory is not None:
        print("Found a trajectory!")
        with open(os.path.join(HERE, 'free_motion_result.json'), 'w') as f:
            json.dump(trajectory, f, cls=DataEncoder, indent=4, sort_keys=True)
    else:
        print("Failed to find a trajectory!")
