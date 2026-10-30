"""Where the robot goes: programs stored on the robot, and the few poses Python still sends.

Absolute moves are robot programs, taught and edited in the Mecademic web portal
and started with ``StartProgram``. Python only knows their names (below):

``fiber<N>_in``
    From any safe pose down to slot N's grip pose. Never drives the gripper:
    Python closes it there on Pick and opens it there on Place.
``fiber<N>_out``
    From slot N's grip pose back to a safe pose. Pick and Place share both
    programs, so a fiber always goes back exactly where it was taken from.
``align_start``
    From a safe pose to the alignment start: the held fiber faces the fixed
    one with a small gap, with the TCP at the fiber tip.
``align_retract``
    From the aligned pose, anywhere inside the alignment window around the
    start, straight back out along the fiber axis to a safe pose.

Rules for the programs:

* Teach them with ``TOOL_TRF`` active. Python sets it before every step, and
  the alignment pivots about it.
* They run with the speeds Python sets (``joint_vel_pct``, ``lin_vel_mm_s``)
  unless they set their own.

Python still sends ``TOOL_TRF`` and the HOME/SHIPMENT joint positions. Until
``POSES_TAUGHT`` is True, every robot motion is refused (see
``cell.prepare_robot``).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator, Tuple

#: Flip to True once TOOL_TRF and the joint positions below are taught.
POSES_TAUGHT = True


@dataclass(frozen=True)
class Pose:
    """A cartesian pose: position in mm, orientation (alpha, beta, gamma) in degrees."""

    x: float
    y: float
    z: float
    alpha: float
    beta: float
    gamma: float

    def __iter__(self) -> Iterator[float]:
        """Unpack as six arguments: ``robot.SetTrf(*pose)``."""
        return iter((self.x, self.y, self.z, self.alpha, self.beta, self.gamma))


@dataclass(frozen=True)
class FiberSlot:
    """One holder of the fiber rack. ``number`` is what the operator sees (1..N)."""

    number: int
    #: Robot program: safe pose → grip pose of this slot.
    program_in: str
    #: Robot program: grip pose of this slot → safe pose.
    program_out: str


# --- Tool --------------------------------------------------------------------

#: Tool reference frame (SetTrf), with the TCP at the tip of the held fiber.
TOOL_TRF = Pose(0 , -8, 14, 0, 0, 180)

# --- Joint positions -----------------------------------------------------------

HOME_JOINTS: Tuple[float, ...] = (0.0, -30.0, 20.0, 0.0, 30.0, 0.0)  # TODO teach
SHIPMENT_JOINTS: Tuple[float, ...] = (0, 0, 0, 0, 0, 0)

# --- Robot programs -------------------------------------------------------------

#: One entry per fiber holder. Add or remove entries to change N; numbers must
#: run 1..N in order.
FIBER_SLOTS: Tuple[FiberSlot, ...] = (
    FiberSlot(1, "fiber1_in", "fiber1_out"),
    FiberSlot(2, "fiber2_in", "fiber2_out"),
    FiberSlot(3, "fiber3_in", "fiber3_out"),
    FiberSlot(4, "fiber4_in", "fiber4_out"),
)

ALIGN_START_PROGRAM = "align_start"
ALIGN_RETRACT_PROGRAM = "align_retract"

INIT_PROGRAM = "init"


if [slot.number for slot in FIBER_SLOTS] != list(range(1, len(FIBER_SLOTS) + 1)):
    raise ValueError("poses.FIBER_SLOTS must be numbered 1..N, in order.")


def fiber_slot(number: int) -> FiberSlot:
    """The slot numbered ``number`` (1..N)."""
    if not 1 <= number <= len(FIBER_SLOTS):
        raise ValueError(
            f"Fiber slot {number} is not defined: poses.FIBER_SLOTS has slots 1..{len(FIBER_SLOTS)}."
        )
    return FIBER_SLOTS[number - 1]


def next_slot_number(number: int) -> int:
    """The slot after ``number``, wrapping from N back to 1."""
    return number % len(FIBER_SLOTS) + 1
