import json
import os
import pybullet_planning as pp

from compas.robots import RobotModel
from compas.data import DataDecoder, json_dumps

from compas_fab.robots import Configuration
from compas_fab.planning.action import FreeMovement, LinearMovement
from compas_fab.planning import AssemblyProcess, SceneState, ToolState, WorkpieceState, RobotState

from compas_fab_pychoreo.client import PyChoreoClient

from state import initialize_process_scene_state, load_robot, set_state

def get_ik_generator(client: PyChoreoClient, robot: RobotModel, group=None, max_attempts=50, options=None):
    options = options or {}

    robot_uid = client.get_robot_pybullet_uid(robot)
    ik_joint_names = robot.get_configurable_joint_names(group=group)
    ik_joints = pp.joints_from_names(robot_uid, ik_joint_names)
    joint_types = robot.get_joint_types_by_names(ik_joint_names)
    sample_fn = pp.get_sample_fn(robot_uid, ik_joints)

    def ik_generator(frame_WCF):
        for iteration in range(max_attempts):
            random_guess_conf = Configuration(joint_values=sample_fn(), joint_types=joint_types, joint_names=ik_joint_names)
            client.set_robot_configuration(robot, random_guess_conf)

            # options['avoid_collisions'] = False
            configuration = client.inverse_kinematics(robot, frame_WCF, start_configuration=random_guess_conf, group=group, options=options)
            if configuration is not None:
                yield configuration

    return ik_generator

def plan_free_movement(client, 
                       robot, 
                       state: SceneState, 
                       movement: FreeMovement, 
                       start_conf=None, end_conf=None, 
                       start_frame=None, end_frame=None,
                       group=None, options=None):
    options = options or {}

    if start_conf is not None:
        start_ik_generator = lambda frame: [start_conf]
    else:
        start_ik_generator = get_ik_generator(client, robot, group)
        assert start_frame is not None

    if end_conf is not None:
        end_ik_generator = lambda frame: [end_conf]
    else:
        end_ik_generator = get_ik_generator(client, robot, group)
        assert end_frame is not None

    set_state(client, robot, state)
    trajectory = None
    for start_conf in start_ik_generator(start_frame):
        for end_conf in end_ik_generator(end_frame):
            options['diagnosis'] = True
            goal_constraint = robot.constraints_from_configuration(end_conf, [0.01], [0.01], group=group)
            trajectory = client.plan_motion(robot, goal_constraint, start_configuration=start_conf,
                            group=group, options=options)
            if trajectory is not None:
                break
        if trajectory is not None:
            break

    return trajectory
    
        
# def plan_linear_movement(client, robot, movement: LinearMovement, start_conf=None, end_conf=None):
    # if start_conf is not None and end_conf is not None:
    #     raise ValueError("Cannot specify both start and end configuration in a linear movement. Problem could be overly constrained. Please specify only one of them.")

    # plan_forward = True
    # if start_conf is not None:
    #     ik_generator = lambda frame: [start_conf]
    # else:
    #     ik_generator = ik sampling generator with a given number of attempts

    # if end_conf is not None:
    #     plan_forward = False
    #     ik_generator = lambda frame: [end_conf]

    # if plan_forward:
    #     target_frames = movement.target_frames
    # else:
    #     target_frames = movement.target_frames[::-1]

    # traj_found = False
    # for start_conf in ik_generator(target_frames[0]):
    #     trajectory = lmp(target_frames[0], start_configuration=start_conf)
    #     if trajectory is not None:
    #         traj_found = True
    #         break

    # if not plan_forward:
    #     trajectory = trajectory[::-1]
    #     # and other cleanup

    # return trajectory

######################


def main():
    HERE = os.path.dirname(__file__)
    with open(os.path.join(HERE, 'process.json'), 'r') as f:
        assembly_process = json.load(f, cls=DataDecoder) #type: AssemblyProcess

    # Print Initial State
    initial_state = assembly_process.get_initial_state()

    with PyChoreoClient(viewer=True) as client:
        initialize_process_scene_state(client, assembly_process)
        robot = load_robot(client, 'abb_crb15000')
        set_state(client, robot, initial_state)
        # pp.wait_if_gui('Initial State')

        for i, movement in enumerate(assembly_process.get_robotic_actions()):
            if isinstance(movement, FreeMovement):
                state = assembly_process.get_intermediate_state(i)
                start_conf = state.robot_state.configuration
                start_frame = state.robot_state.frame

                end_conf = movement.fixed_configuration
                end_frame = movement.robot_target
                # acm = movement.allowed_collision_pairs

                trajectory = plan_free_movement(client, robot, 
                                                state, movement, 
                                                start_conf, end_conf, 
                                                start_frame, end_frame,
                                                group=None, options=None)
                if trajectory is not None:
                    for conf in trajectory.points:
                        client.set_robot_configuration(robot, conf)
                        pp.wait_if_gui('Step traj')
                else:
                    raise ValueError("Trajectory not found")

        # for i in range(len(assembly_process.actions)):
        #     state = assembly_process.get_intermediate_state(i)
        #     set_state(client, robot, state)
        #     pp.wait_if_gui(f'State {i}')

if __name__ == '__main__':
    main()