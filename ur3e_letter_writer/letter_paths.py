"""
Letter geometry for the UR3/UR3e "letter writer" demo.

Each letter is described as a list of *strokes*. A stroke is a polyline the
pen stays down for; between strokes the pen lifts, moves, and comes back
down. Coordinates are given in a normalized unit square:

    u in [0, 1]  -> horizontal position within the letter (left to right)
    v in [0, 1]  -> vertical position within the letter (bottom to top)

`build_letter_waypoints()` turns those normalized strokes into an ordered
list of absolute 3D points (with a pen-down/pen-up flag) in whatever plane
and size the caller asks for, ready to hand to MoveIt as Cartesian
waypoints.

Only straight-line segments are used (no curves), so every letter is a
"single-stroke-per-polyline, block/skeleton" style letterform -- simple,
robust, and easy to verify offline (see the offline IK check used while
designing this package).
"""

from dataclasses import dataclass
from typing import List, Sequence, Tuple

Point2D = Tuple[float, float]
Stroke = Sequence[Point2D]

# Bounding-box skeleton strokes for each supported letter, normalized to
# the unit square [0,1] x [0,1] (origin at bottom-left of the letter).
LETTER_STROKES = {
    'T': [
        [(0.0, 1.0), (1.0, 1.0)],   # top bar
        [(0.5, 1.0), (0.5, 0.0)],   # stem
    ],
    'L': [
        [(0.0, 1.0), (0.0, 0.0), (1.0, 0.0)],
    ],
    'V': [
        [(0.0, 1.0), (0.5, 0.0), (1.0, 1.0)],
    ],
    'M': [
        [(0.0, 0.0), (0.0, 1.0), (0.5, 0.45), (1.0, 1.0), (1.0, 0.0)],
    ],
    'N': [
        [(0.0, 0.0), (0.0, 1.0), (1.0, 0.0), (1.0, 1.0)],
    ],
    'A': [
        [(0.0, 0.0), (0.5, 1.0), (1.0, 0.0)],
        [(0.22, 0.38), (0.78, 0.38)],
    ],
    'B': [
        [(0.0, 0.0), (0.0, 1.0)],
        [(0.0, 1.0), (0.65, 1.0), (0.8, 0.85), (0.8, 0.65), (0.65, 0.5), (0.0, 0.5)],
        [(0.0, 0.5), (0.7, 0.5), (0.85, 0.35), (0.85, 0.15), (0.7, 0.0), (0.0, 0.0)],
    ],
}


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    pen_down: bool
    label: str


def available_letters() -> List[str]:
    return sorted(LETTER_STROKES.keys())


def build_letter_waypoints(
    letter: str,
    x_center: float,
    y_center: float,
    width: float,
    height: float,
    z_draw: float,
    z_lift: float,
) -> List[Waypoint]:
    """
    Build the full ordered waypoint list (approach -> draw -> lift, per
    stroke, for every stroke of `letter`) in the caller's Cartesian frame.

    The letter is centered on (x_center, y_center) and scaled to
    `width` x `height`. `z_draw` is the pen-down height, `z_lift` the
    pen-up (retreat) height -- both in the same frame as x_center/y_center.
    """
    letter = letter.strip().upper()
    if letter not in LETTER_STROKES:
        raise ValueError(
            f"Letter '{letter}' is not defined. Available letters: "
            f"{available_letters()}"
        )

    def to_xy(u: float, v: float) -> Tuple[float, float]:
        return (
            x_center + (u - 0.5) * width,
            y_center + v * height,
        )

    waypoints: List[Waypoint] = []
    strokes = LETTER_STROKES[letter]

    x0, y0 = to_xy(*strokes[0][0])
    waypoints.append(Waypoint(x0, y0, z_lift, False, 'start_approach'))

    for s_idx, stroke in enumerate(strokes):
        pts = [to_xy(u, v) for (u, v) in stroke]

        # move (pen up) to above the stroke's first point
        waypoints.append(Waypoint(pts[0][0], pts[0][1], z_lift, False,
                                   f'stroke{s_idx}_approach'))
        # pen down
        waypoints.append(Waypoint(pts[0][0], pts[0][1], z_draw, True,
                                   f'stroke{s_idx}_touch_down'))
        # draw the polyline
        for p_idx, (x, y) in enumerate(pts[1:], start=1):
            waypoints.append(Waypoint(x, y, z_draw, True,
                                       f'stroke{s_idx}_draw{p_idx}'))
        # pen up
        waypoints.append(Waypoint(pts[-1][0], pts[-1][1], z_lift, False,
                                   f'stroke{s_idx}_lift'))

    waypoints.append(Waypoint(x0, y0, z_lift, False, 'end_retreat'))
    return waypoints
