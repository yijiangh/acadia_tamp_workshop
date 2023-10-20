from typing import List, Tuple
from compas_fab.planning.action import RoboticMovement
from compas_fab.planning import SceneState

def add_acm(client, state: SceneState, acm_name: str, allowed_collision_pairs:List[Tuple[str, str]]=None):
    allowed_collision_pairs = allowed_collision_pairs or []

    # add ACM of the current action
    acm_name = '_action_acm'
    # ACM between the workpiece and the tool
    for wp_id, wp_state in state.workpiece_states.items():
        if wp_state.attached_to_tool_id is not None:
            for tool_id, tool_state in state.tool_states.items():
                if tool_state.attached_to_robot:
                    client.extra_disabled_collision_links[acm_name].add(
                            ((client._get_bodies(wp_id)[0], None), (client._get_bodies(tool_id)[0], None))
                            )
    # Extra provided ACM
    for id1, id2 in allowed_collision_pairs:
        client.extra_disabled_collision_links[acm_name].add(
                ((client._get_bodies(id1)[0], None), (client._get_bodies(id2)[0], None))
                )

