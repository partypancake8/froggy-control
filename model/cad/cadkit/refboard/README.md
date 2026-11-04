# cadkit reference board

What "finished" looks like, per part type. Before drawing a part, open the listing for its type,
name the two or three traits you are copying in the part's docstring, and check the render against
the listing photo. No images are stored here; the URLs are enough. Traits are what a stranger
notices in a catalogue photo, written so a reviewer can answer yes or no.

## Bench instruments (whole-machine look)

| Reference | URL | Traits to copy |
|---|---|---|
| Eppendorf Centrifuge 5424 | https://www.eppendorf.com/us-en/eShop-Products/Centrifugation/Microcentrifuges/Centrifuge-5424-5424R-p-PF-17305 | one continuous shell; rotor hidden behind a lid; one control face; nothing hanging off the sides |
| Bambu Lab P1S | https://bambulab.com/en/p1 | continuous black panels; framed clear window; no visible wiring; one accent colour; fasteners only where serviced |
| Bambu Lab X1C | https://bambulab.com/en/x1 | flush panel seams with equal gaps; rounded vertical corners; feet tucked under |
| Thorlabs cage system | https://www.thorlabs.com/navigation.cfm?guide_id=2004 | flat anodised plates; aligned fastener grids; black cap heads; every part reads as a catalogue part |
| Thorlabs breadboards | https://www.thorlabs.com/navigation.cfm?guide_id=2033 | one hole pitch everywhere; chamfered edges; clean datum faces |

## Enclosures and panels

| Reference | URL | Traits to copy |
|---|---|---|
| Hammond 1455 extruded aluminium | https://www.hammfg.com/electronics/small-case/extruded/1455 | end panels with even screw inset; board slides in rails; radiused extrusion corners |
| Hammond 1591 ABS | https://www.hammfg.com/electronics/small-case/plastic/1591 | lid lip that wraps the base; internal bosses tied to walls with ribs; radius on every outside edge |
| Takachi aluminium cases | https://www.takachi-enclosure.com/ | panel cut-outs with a bezel or lip, never a raw cut edge; grouped controls on one centreline |
| Takachi plastic cases | https://www.takachi-enclosure.com/ | soft-radius shells; countersunk or recessed screws on show faces |

## Brackets, frames, mounts

| Reference | URL | Traits to copy |
|---|---|---|
| Misumi brackets (HBLFS and similar) | https://us.misumi-ec.com/vona2/mech/M1500000000/M1501000000/ | gussets that leave the screw head clear; bend radius equal to thickness; hole 2 x d from the edge |
| Misumi aluminium frame parts | https://us.misumi-ec.com/vona2/mech/M1500000000/ | one fastener family per frame; end caps on every visible profile end |
| OpenBuilds V-slot parts | https://openbuildspartstore.com/v-slot-linear-rail/ | laser plates with R corners and arms blended by tangent arcs; equal-pitch hole rows |
| OpenBuilds plates | https://openbuildspartstore.com/plates/ | symmetric outlines; slots instead of drifting holes for adjustment |

## Printed parts (open-source prints done well)

| Reference | URL | Traits to copy |
|---|---|---|
| Prusa printer printed parts | https://github.com/prusa3d/Original-Prusa-MK4 | printed flat on the largest face; elephant-foot chamfer; ribs instead of solid blocks; nut traps with bridging layers |
| Voron Design printed parts | https://github.com/VoronDesign/Voron-2 | stated print orientation per part; heat-set inserts in 2.4 mm walls; accent colour on all printed parts and nowhere else |

## How to use it

1. Pick the row for the part type and write the copied traits into the part docstring.
2. Build with `cadkit.idioms`, then run `cadkit lint` and `cadkit views`.
3. Put the iso render next to the listing photo. If the part would look wrong in that listing,
   it is not finished.
