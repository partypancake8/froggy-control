"""fp7_controller_carrier: PETG tray for the control electronics on the plinth floor (v7).

84 x 56 x 4 plate on the bottom panel at Z -92..-88 (centre X -66, Y 92), held by
4 x M3 x 8 from under the panel into heat-set inserts at its corners. Four
standoff bosses (9 OD, 4 tall, M2.5 pilot holes, 47 x 11.4 pattern) carry the
Raspberry Pi Pico; the buck module and the delay-off relay sit on the plate.
"""
from build123d import Align, Box, Cylinder, Location

from lib import hardware as HW
from parts._util import fillet_safe, one

LX, LY, T = 84.0, 56.0, 4.0
CENTRE = (-66.0, 92.0)
Z0 = -92.0
INSERTS = [(sx * 36.0, sy * 10.0) for sx in (1, -1) for sy in (1, -1)]
PICO_C = (-26.0, 0.0)
BOSSES = [(PICO_C[0] + sx * 5.7, sy * 23.5) for sx in (1, -1) for sy in (1, -1)]
BOSS_D, BOSS_H = 9.0, 4.0
BUCK_C, RELAY_C = (16.0, 14.0), (16.0, -14.0)

SPEC = dict(
    name="fp7_controller_carrier",
    function="Carries the Pico controller, the 24 V to 5 V buck and the delay-off relay on the plinth floor.",
    mates=[dict(part="s_panel_bottom", feature="underside Z -92, 4 x M3 x 8 from below into inserts", kind="heat_set_insert x4"),
           dict(part="v_pico", feature="four 9 OD bosses, 47 x 11.4", kind="seat")],
    loads=[dict(where=[0, 0, 0], direction=[0, 0, -1], newton=2, note="boards")],
    print_face="-z (underside)",
    print_rotate=(0, 0, 0),
    material="PETG",
    nonstructural=[],
)


def _local():
    b = Box(LX, LY, T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    vert = [e for e in b.edges() if e.geom_type.name == "LINE" and abs(e.tangent_at(0).Z) > 0.99]
    b, _ = fillet_safe(b, 4.0, vert)
    for sy in (1, -1):                       # one 20.4 x 9 standoff pad per pair of Pico holes
        b = b + Box(11.4 + BOSS_D, BOSS_D, BOSS_H + 0.5, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((PICO_C[0], sy * 23.5, T - 0.5)))
    for x, y in BOSSES:
        b = b - Cylinder(1.1, BOSS_H, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(Location((x, y, T + BOSS_H)))
    for x, y in INSERTS:
        b = b - Cylinder(HW.INSERT["M3"]["pocket_d"] / 2, 40).moved(Location((x, y, 0)))
    return one(b)


def build():
    b = _local().moved(Location((CENTRE[0], CENTRE[1], Z0)))
    b.label = "fp7_controller_carrier"
    return b


def world(x, y, dz=0.0):
    return (CENTRE[0] + x, CENTRE[1] + y, Z0 + dz)
