import json
from json import tool
import os
from colorama import reinit

from compas.data import DataDecoder, json_dumps
from compas.robots import RobotModel

from compas_fab.planning import AssemblyProcess, SceneState
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab.robots import AttachedCollisionMesh, Configuration, CollisionMesh, Robot
from compas_fab.robots import RobotSemantics

import pybullet_planning as pp

HERE = os.path.dirname(__file__)
TOOL_EXPORT_DIR = os.path.join(HERE, 'tools')
ROBOT_MODEL_DIR = os.path.join(HERE, '..', '..', 'robots')

def load_robot(client: PyChoreoClient, robot_name: str):
    if robot_name == 'abb_crb15000':
        urdf_filename = os.path.join(ROBOT_MODEL_DIR, 'abb_crb15000_support', "urdf", "crb15000_5_95.urdf")
        srdf_filename = os.path.join(ROBOT_MODEL_DIR, 'abb_crb15000_support', "srdf", "crb15000_5_95.srdf")
    else:
        raise NotImplementedError(robot_name)

    with pp.LockRenderer():
        robot = client.load_robot(urdf_filename)
        client.load_semantics(robot, srdf_filename)

    return robot

def initialize_process_scene_state(client: PyChoreoClient, process: AssemblyProcess, options=None):
    options = options or {}

    # initialize all geometries in the scene
    # * workpieces
    for wp_id, workpiece in process.workpieces.items():
        for i, wp_mesh in enumerate(workpiece.mesh):
            # in case the mesh is not a triangle mesh
            # wp_mesh = wp_mesh.copy()
            # mesh_quads_to_triangles(wp_mesh)
            cm = CollisionMesh(wp_mesh, wp_id)
            if i == 0:
                client.add_collision_mesh(cm, {'color': pp.GREY})
            else:
                client.append_collision_mesh(cm, {'color': pp.GREY})

    # * tools
    with pp.HideOutput():
        tool_robot = pp.load_pybullet(process.tool.urdf_file_path, fixed_base=False)
        client.collision_objects[process.tool_id] = [tool_robot]

def set_state(client: PyChoreoClient, robot: RobotModel, state: SceneState, options=None):
    options = options or {}
    debug = options.get('debug', False)
    tool0_link_name = robot.get_end_effector_link_name()

    def update_attached_state(object_id, object_state):
        object_names, status = client.get_object_names_and_status(object_id)
        assert status != 'not_exist', 'Object set object id ({}) | body names: {} as attached in scene but object not added to the scene!'.format(object_id, object_names)

        if object_state.attached_to_robot:
            client.add_attached_collision_mesh(
                AttachedCollisionMesh(CollisionMesh(None, tool_id),
                                      tool0_link_name, touch_links=[]),
                options={'robot': robot,
                        #  'attached_child_link_name': tool_attach_link_Name,
                         'parent_link_from_child_link_transformation' : tool_state.attached_to_robot_grasp,
                         })
        else:
            # if the current status in the client is not attached, detach it
            if status == 'attached_object':
                client.detach_attached_collision_mesh(tool_id)

    with pp.LockRenderer(not debug):
        # * Robot
        client.set_robot_configuration(robot, state.robot_state.configuration)

        # * Tools
        for tool_id, tool_state in state.tool_states.items():
            client.set_object_frame(tool_id, tool_state.frame)
            client.set_tool_configuration(tool_id, tool_state.configuration)
            update_attached_state(tool_id, tool_state)

        # * Workpieces
        for wp_id, wp_state in state.workpiece_states.items():
            client.set_object_frame(wp_id, wp_state.frame)
            update_attached_state(wp_id, wp_state)

#####################

def main():
    # Load Process File from JSON

    with open(os.path.join(HERE, 'process.json'), 'r') as f:
        assembly_process = json.load(f, cls=DataDecoder) #type: AssemblyProcess

    # Print Initial State
    initial_state = assembly_process.get_initial_state()

    with PyChoreoClient(viewer=True) as client:
        initialize_process_scene_state(client, assembly_process)
        robot = load_robot(client, 'abb_crb15000')
        set_state(client, robot, initial_state)

        pp.wait_if_gui()

        # # Print last state
        # for i in range(len(assembly_process.actions)):
        #     state = assembly_process.get_intermediate_state()

if __name__ == '__main__':
    main()