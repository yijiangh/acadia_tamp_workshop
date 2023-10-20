import os
import json
import logging
from tracemalloc import start
import pybullet_planning as pp
from termcolor import colored, cprint
from typing import List, Tuple

from compas.robots import RobotModel
from compas.data import DataDecoder, DataEncoder, json_dumps

from compas_fab.robots import Configuration
from compas_fab.planning.action import FreeMovement, LinearMovement
from compas_fab.planning import AssemblyProcess, SceneState, ToolState, WorkpieceState, RobotState

from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.conversions import pose_from_frame

from state import initialize_process_scene_state, load_robot, set_state
from utils import LOGGER, get_tolerances

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
            # options['diagnosis'] = True
            configuration = client.inverse_kinematics(robot, frame_WCF, start_configuration=random_guess_conf, group=group, options=options)
            if configuration is not None:
                LOGGER.debug('IK found in {} iterations'.format(iteration))
                yield configuration
        else:
            LOGGER.debug('IK not found in {} iterations'.format(max_attempts))

    return ik_generator

def plan_free_movement(client, 
                       robot, 
                       state: SceneState, 
                       allowed_collision_pairs: List[Tuple[str, str]], 
                       start_conf=None, end_conf=None, 
                       start_frame=None, end_frame=None,
                       group=None, options=None):
    options = options or {}
    max_attempts = 100

    if start_conf is not None:
        def _start_ik_generator(frame):
            yield start_conf
        start_ik_generator = _start_ik_generator
    else:
        start_ik_generator = get_ik_generator(client, robot, group, max_attempts)
        assert start_frame is not None

    if end_conf is not None:
        def _end_ik_generator(frame):
            yield end_conf
        end_ik_generator = _end_ik_generator
    else:
        end_ik_generator = get_ik_generator(client, robot, group, max_attempts)
        assert end_frame is not None

    set_state(client, robot, state, options)

    # ? should ACM be part of set_state?
    acm_name = '_fmp_acm'
    # TODO this ACM should be handled by the Process
    # Hardcoded ACM between the workpiece and the tool
    for wp_id, wp_state in state.workpiece_states.items():
        if wp_state.attached_to_tool_id is not None:
            for tool_id, tool_state in state.tool_states.items():
                if tool_state.attached_to_robot:
                    client.extra_disabled_collision_links[acm_name].add(
                            ((client._get_bodies(wp_id)[0], None), (client._get_bodies(tool_id)[0], None))
                            )
    # TODO allowed_collision_pairs

    trajectory = None
    for start_conf in start_ik_generator(start_frame):
        for end_conf in end_ik_generator(end_frame):
            # TODO remove diagnosis
            # options['diagnosis'] = True
            # client.set_robot_configuration(robot, start_conf)
            # pp.wait_if_gui('Start conf')
            # client.set_robot_configuration(robot, end_conf)
            # pp.wait_if_gui('End conf')

            goal_constraint = robot.constraints_from_configuration(end_conf, [0.01], [0.01], group=group)
            trajectory = client.plan_motion(robot, goal_constraint, start_configuration=start_conf, group=group, options=options)

            if trajectory is not None:
                break
        if trajectory is not None:
            break

    # clean up ACM
    if acm_name in client.extra_disabled_collision_links:
        del client.extra_disabled_collision_links[acm_name]

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

    viewer = 0
    write = 1
    debug = 1
    diagnosis = 0

    logging_level = logging.DEBUG if debug else logging.INFO
    LOGGER.setLevel(logging_level)

    options = {
        'debug': debug,
        'diagnosis': diagnosis
        }

    with PyChoreoClient(viewer=viewer) as client:
        initialize_process_scene_state(client, assembly_process)
        robot = load_robot(client, 'abb_crb15000')
        options.update(get_tolerances(robot, super_res=True))

        set_state(client, robot, assembly_process.get_initial_state(), options)
        # pp.wait_if_gui('Initial State')

        # actions = assembly_process.actions
        # if True:
        #     action_index = 52
        #     action = actions[action_index]

        for action in assembly_process.get_robotic_actions():
            trajectory = None
            if isinstance(action, FreeMovement):
                action_index = assembly_process.actions.index(action)
                LOGGER.debug(colored('Planning Free Movement {}'.format(action_index), 'cyan'))

                state = assembly_process.get_intermediate_state(action_index)

                start_conf = assembly_process.get_action_starting_configuration(action)
                start_frame = state.robot_state.frame

                end_conf = assembly_process.get_action_ending_configuration(action)
                end_frame = action.robot_target

                if debug and viewer:
                    pp.draw_pose(pose_from_frame(start_frame), length=0.1)
                    pp.add_text('Start', start_frame.point, color=(1,0,0))
                    pp.draw_pose(pose_from_frame(end_frame), length=0.1)
                    pp.add_text('End', end_frame.point, color=(1,0,0))

                trajectory = plan_free_movement(client, robot, 
                                                state, action, 
                                                start_conf, end_conf, 
                                                start_frame, end_frame,
                                                group=None, options=options)

            if trajectory is not None:
                LOGGER.debug(colored('Trajectory found for movement {}'.format(action_index), 'green'))
                action.planned_trajectory = trajectory

                # replay for debugging
                if debug and viewer:
                    set_state(client, robot, state, options)
                    for conf in trajectory.points:
                        client.set_robot_configuration(robot, conf)
                        pp.wait_if_gui('Step traj')
            else:
                LOGGER.debug(colored('Trajectory NOT found for movement {}'.format(action_index), 'red'))
                # raise ValueError("Trajectory not found")

    if write:
        with open(os.path.join(HERE, 'process.json'), 'w') as f:
            json.dump(assembly_process, f, cls=DataEncoder, indent=4, sort_keys=True)
        LOGGER.info('Process saved to process.json')

if __name__ == '__main__':
    main()