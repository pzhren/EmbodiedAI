import os
from typing import Any, Dict, List, Optional, Sequence, Tuple
import imageio
import numpy as np
import scipy.ndimage
import cv2


AGENT_SPRITE = imageio.imread("100x100.png")
    
AGENT_SPRITE = np.ascontiguousarray(np.flipud(AGENT_SPRITE))

MAP_INVALID_POINT = 0
MAP_VALID_POINT = 1
MAP_BORDER_INDICATOR = 2
MAP_SOURCE_POINT_INDICATOR = 4
MAP_TARGET_POINT_INDICATOR = 6
MAP_SHORTEST_PATH_COLOR = 7
MAP_VIEW_POINT_INDICATOR = 8
MAP_TARGET_BOUNDING_BOX = 9
TOP_DOWN_MAP_COLORS = np.full((256, 3), 150, dtype=np.uint8)
TOP_DOWN_MAP_COLORS[10:] = cv2.applyColorMap(
    np.arange(246, dtype=np.uint8), cv2.COLORMAP_JET
).squeeze(1)[:, ::-1]
TOP_DOWN_MAP_COLORS[MAP_INVALID_POINT] = [255, 255, 255]  # White
TOP_DOWN_MAP_COLORS[MAP_VALID_POINT] = [150, 150, 150]  # Light Grey
TOP_DOWN_MAP_COLORS[MAP_BORDER_INDICATOR] = [50, 50, 50]  # Grey
TOP_DOWN_MAP_COLORS[MAP_SOURCE_POINT_INDICATOR] = [0, 0, 200]  # Blue
TOP_DOWN_MAP_COLORS[MAP_TARGET_POINT_INDICATOR] = [200, 0, 0]  # Red
TOP_DOWN_MAP_COLORS[MAP_SHORTEST_PATH_COLOR] = [0, 200, 0]  # Green
TOP_DOWN_MAP_COLORS[MAP_VIEW_POINT_INDICATOR] = [245, 150, 150]  # Light Red
TOP_DOWN_MAP_COLORS[MAP_TARGET_BOUNDING_BOX] = [0, 175, 0]  # Green


def paste_overlapping_image(
    background: np.ndarray,
    foreground: np.ndarray,
    location: Tuple[int, int],
    mask: Optional[np.ndarray] = None,
):
    r"""Composites the foreground onto the background dealing with edge
    boundaries.
    Args:
        background: the background image to paste on.
        foreground: the image to paste. Can be RGB or RGBA. If using alpha
            blending, values for foreground and background should both be
            between 0 and 255. Otherwise behavior is undefined.
        location: the image coordinates to paste the foreground.
        mask: If not None, a mask for deciding what part of the foreground to
            use. Must be the same size as the foreground if provided.
    Returns:
        The modified background image. This operation is in place.
    """
    assert mask is None or mask.shape[:2] == foreground.shape[:2]
    foreground_size = foreground.shape[:2]
    min_pad = (
        max(0, foreground_size[0] // 2 - location[0]),
        max(0, foreground_size[1] // 2 - location[1]),
    )

    max_pad = (
        max(
            0,
            (location[0] + (foreground_size[0] - foreground_size[0] // 2))
            - background.shape[0],
        ),
        max(
            0,
            (location[1] + (foreground_size[1] - foreground_size[1] // 2))
            - background.shape[1],
        ),
    )

    background_patch = background[
        (location[0] - foreground_size[0] // 2 + min_pad[0]) : (
            location[0]
            + (foreground_size[0] - foreground_size[0] // 2)
            - max_pad[0]
        ),
        (location[1] - foreground_size[1] // 2 + min_pad[1]) : (
            location[1]
            + (foreground_size[1] - foreground_size[1] // 2)
            - max_pad[1]
        ),
    ]
    foreground = foreground[
        min_pad[0] : foreground.shape[0] - max_pad[0],
        min_pad[1] : foreground.shape[1] - max_pad[1],
    ]
    if foreground.size == 0 or background_patch.size == 0:
        # Nothing to do, no overlap.
        return background

    if mask is not None:
        mask = mask[
            min_pad[0] : foreground.shape[0] - max_pad[0],
            min_pad[1] : foreground.shape[1] - max_pad[1],
        ]

    if foreground.shape[2] == 4:
        # Alpha blending
        foreground = (
            background_patch.astype(np.int32) * (255 - foreground[:, :, [3]])
            + foreground[:, :, :3].astype(np.int32) * foreground[:, :, [3]]
        ) // 255
    if mask is not None:
        background_patch[mask] = foreground[mask]
    else:
        background_patch[:] = foreground
    return background

def draw_agent(
    image: np.ndarray,
    agent_center_coord: Tuple[int, int],
    agent_rotation: float,
    agent_radius_px: int = 5,
) -> np.ndarray:
    r"""Return an image with the agent image composited onto it.
    Args:
        image: the image onto which to put the agent.
        agent_center_coord: the image coordinates where to paste the agent.
        agent_rotation: the agent's current rotation in radians.
        agent_radius_px: 1/2 number of pixels the agent will be resized to.
    Returns:
        The modified background image. This operation is in place.
    """

    # Rotate before resize to keep good resolution.
    rotated_agent = scipy.ndimage.interpolation.rotate(
        AGENT_SPRITE, agent_rotation * 180 / np.pi
    )
    # Rescale because rotation may result in larger image than original, but
    # the agent sprite size should stay the same.
    initial_agent_size = AGENT_SPRITE.shape[0]
    new_size = rotated_agent.shape[0]
    agent_size_px = max(
        1, int(agent_radius_px * 2 * new_size / initial_agent_size)
    )
    resized_agent = cv2.resize(
        rotated_agent,
        (agent_size_px, agent_size_px),
        interpolation=cv2.INTER_LINEAR,
    )
    paste_overlapping_image(image, resized_agent, agent_center_coord)
    return image


def colorize_topdown_map(
    top_down_map: np.ndarray,
    fog_of_war_mask: Optional[np.ndarray] = None,
    fog_of_war_desat_amount: float = 0.5,
) -> np.ndarray:
    r"""Convert the top down map to RGB based on the indicator values.
    Args:
        top_down_map: A non-colored version of the map.
        fog_of_war_mask: A mask used to determine which parts of the
            top_down_map are visible
            Non-visible parts will be desaturated
        fog_of_war_desat_amount: Amount to desaturate the color of unexplored areas
            Decreasing this value will make unexplored areas darker
            Default: 0.5
    Returns:
        A colored version of the top-down map.
    """
    _map = TOP_DOWN_MAP_COLORS[top_down_map]

    if fog_of_war_mask is not None:
        fog_of_war_desat_values = np.array([[fog_of_war_desat_amount], [1.0]])
        # Only desaturate things that are valid points as only valid points get revealed
        desat_mask = top_down_map != MAP_INVALID_POINT

        _map[desat_mask] = (
            _map * fog_of_war_desat_values[fog_of_war_mask]
        ).astype(np.uint8)[desat_mask]

    return _map


def colorize_draw_agent_and_fit_to_height(
    topdown_map_info: Dict[str, Any], output_height: int
):
    r"""Given the output of the TopDownMap measure, colorizes the map, draws the agent,
    and fits to a desired output height

    :param topdown_map_info: The output of the TopDownMap measure
    :param output_height: The desired output height
    """
    top_down_map = topdown_map_info["map"]
    # top_down_map = colorize_topdown_map(
    #     top_down_map, topdown_map_info["fog_of_war_mask"]
    # )
    for agent_idx in range(len(topdown_map_info["agent_map_coord"])):
        map_agent_pos = topdown_map_info["agent_map_coord"][agent_idx]
        map_agent_angle = topdown_map_info["agent_angle"][agent_idx]
        top_down_map = draw_agent(
            image=top_down_map,
            agent_center_coord=map_agent_pos,
            agent_rotation=map_agent_angle,
            agent_radius_px=min(top_down_map.shape[0:2]) // 32,
        )

    if top_down_map.shape[0] > top_down_map.shape[1]:
        top_down_map = np.rot90(top_down_map, 1)

    # scale top down map to align with rgb view
    old_h, old_w, _ = top_down_map.shape
    top_down_height = output_height
    top_down_width = int(float(top_down_height) / old_h * old_w)
    # cv2 resize (dsize is width first)
    top_down_map = cv2.resize(
        top_down_map,
        (top_down_width, top_down_height),
        interpolation=cv2.INTER_CUBIC,
    )

    return top_down_map
