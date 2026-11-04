"""Part kinds from names: printed, laser, machined, vendor, fastener, unknown.

Prefixes win (fp_ / fp9_ printed, lp_ laser, mp_ machined, v_ vendor); otherwise keywords in the
readable name: PETG / PLA / ASA / printed -> printed; '<n> mm aluminium|acrylic|steel|ABS', 'laser',
'sheet' -> laser; a leading thread size (M3 x 8, M5 T-nut) or screw / nut / washer / insert words ->
fastener; a vendor-style part number after a comma (', goBILDA 1614-0016') -> vendor.
"""
import re

PREFIX = (("fp9_", "printed"), ("fp_", "printed"), ("lp_", "laser"), ("mp_", "machined"), ("v_", "vendor"))
COLOURS = {"printed": (0.95, 0.62, 0.15), "laser": (0.78, 0.80, 0.83), "machined": (0.70, 0.72, 0.76),
           "vendor": (0.30, 0.32, 0.36), "fastener": (0.12, 0.12, 0.13), "unknown": (0.62, 0.66, 0.70)}

_PRINTED = re.compile(r"(?i)(^|[\s_,])(petg|pla|asa|tpu|printed|3d[\s_-]?print)($|[\s_,])")
_LASER = re.compile(r"(?i)(\d+([._]\d+)?[\s_]*mm[\s_]+(aluminium|aluminum|acrylic|steel|abs|sheet)|laser|[\s_]sheet($|[\s_,]))")
_FASTENER = re.compile(r"(?i)^(m\d+([._]\d)?[\s_]*x[\s_]*\d|m\d+([._]\d)?[\s_]+(t-nut|nut|nyloc|washer|insert|heat|"
                       r"screw|bolt|bhcs|shcs|fhcs|button|socket|countersunk|threaded)|(screw|bolt|nut|washer|t-nut|"
                       r"insert|rivet)s?([\s_]|$))")
_VENDOR = re.compile(r",[\s_]*[A-Za-z][A-Za-z0-9-]*[\s_]+[A-Z0-9][A-Z0-9-]{3,}")


def kind_of(name, path=(), override=None):
    if override:
        return override
    n = name or ""
    low = n.lower()
    for pre, k in PREFIX:
        if low.startswith(pre):
            return k
    if _FASTENER.search(n):
        return "fastener"
    if _PRINTED.search(n):
        return "printed"
    if _LASER.search(n):
        return "laser"
    if _VENDOR.search(n):
        return "vendor"
    # a leaf inside a vendor-named group (e.g. 'Hall sensor, Joy-it KY-003' / 'Breakout board')
    for anc in reversed(list(path)[1:]):
        if _FASTENER.search(anc or "") or (anc or "").lower() == "fasteners":
            return "fastener"
        if _VENDOR.search(anc or ""):
            return "vendor"
    return "unknown"
