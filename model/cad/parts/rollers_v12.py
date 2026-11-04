"""Official Creality STEP wheel assembly, local horizontal X axle, centered at origin.
Original source coordinates preserved in vendor files; only rigid placements below.
"""
import copy
from pathlib import Path
from functools import lru_cache
from build123d import import_step, Location, Axis
ROOT=Path(__file__).resolve().parents[1]/"vendor/motion/creality_ender_vwheel"
@lru_cache(None)
def _kit():
    wheel=import_step(str(ROOT/"Rollers.step")).solids()[0]
    wheel=wheel.moved(Location((-15.5,35.5,13.45))).rotate(Axis.Y,90)
    bearings=import_step(str(ROOT/"625pillow.step")).solids()
    shim=import_step(str(ROOT/"M5 washer1.step")).solids()[0].rotate(Axis.Y,90).moved(Location((-.5,0,0)))
    return [wheel]+[copy.deepcopy(b).moved(Location((x,0,0))) for x in (-3,3) for b in bearings]+[shim]
def kit():
    return [copy.deepcopy(s) for s in _kit()]
