"""Intent store: Sawyer's verdicts, verbatim, so none is lost between iterations.

docs/intent/intent.json  {"intents": [entry, ...]}; entry:
  id           "I001".. (stable)
  date         YYYY-MM-DD he said it
  verbatim     his exact words, never paraphrased
  subject      what it is about (part, view, decision)
  commit       the commit he was looking at (short sha) or null
  image        docs/intent/images/<file> (his screenshot, copied in) or null
  status       open | addressed | standing-rule
  addressed_in commit that addressed it (status addressed), or null
  rule         DESIGN-LANGUAGE rule ids it maps to, e.g. ["DL1", "DL2"]
  note         one line of context (progress, what replaced what)

docs/intent/REJECTED.md is generated from it (`bin/cad intent md`, or --md on any action): the
gallery every brief links and every reviewer reads. Edit the JSON through `bin/cad intent`, never by hand.
"""
import json
import os
import re
import shutil
import time

STATUSES = ("open", "addressed", "standing-rule")


def paths(root):
    d = os.path.join(root, "docs", "intent")
    return d, os.path.join(d, "intent.json"), os.path.join(d, "images"), os.path.join(d, "REJECTED.md")


def load(root):
    _, p, _, _ = paths(root)
    if not os.path.exists(p):
        return {"intents": []}
    return json.load(open(p))


def save(root, data):
    d, p, _, _ = paths(root)
    os.makedirs(d, exist_ok=True)
    with open(p, "w") as fh:
        json.dump(data, fh, indent=1, ensure_ascii=False)
        fh.write("\n")


def _slug(s):
    return re.sub(r"[^a-z0-9]+", "-", s.lower())[:40].strip("-") or "image"


def add(root, verbatim, subject="", commit=None, image=None, status="open", date=None, rule=None, note="",
        addressed_in=None):
    if status not in STATUSES:
        raise ValueError(f"status must be one of {STATUSES}")
    data = load(root)
    n = max([int(e["id"][1:]) for e in data["intents"]] + [0]) + 1
    iid = f"I{n:03d}"
    date = date or time.strftime("%Y-%m-%d")
    rel = None
    if image:
        if not os.path.exists(image):
            raise FileNotFoundError(image)
        _, _, imgdir, _ = paths(root)
        os.makedirs(imgdir, exist_ok=True)
        ext = os.path.splitext(image)[1].lower() or ".png"
        fn = f"{iid}-{date}-{_slug(subject or verbatim)}{ext}"
        shutil.copy2(image, os.path.join(imgdir, fn))
        rel = f"docs/intent/images/{fn}"
    e = dict(id=iid, date=date, verbatim=verbatim, subject=subject, commit=commit, image=rel, status=status,
             addressed_in=addressed_in, rule=list(rule or []), note=note)
    data["intents"].append(e)
    save(root, data)
    return e


def resolve(root, iid, commit=None, status="addressed", note=None):
    if status not in STATUSES:
        raise ValueError(f"status must be one of {STATUSES}")
    data = load(root)
    for e in data["intents"]:
        if e["id"] == iid:
            e["status"] = status
            if commit:
                e["addressed_in"] = commit
            if note:
                e["note"] = note
            save(root, data)
            return e
    raise KeyError(f"no intent {iid}")


def markdown(root):
    data = load(root)
    _, _, _, md = paths(root)
    L = ["# Sawyer's verdicts (generated)", "",
         "Generated from `docs/intent/intent.json` by `bin/cad intent md`; do not edit by hand. Every brief quotes",
         "the verdicts it touches, verbatim, with the image; every reviewer reads this page first.", ""]
    order = {"open": 0, "standing-rule": 1, "addressed": 2}
    heads = {"open": "Open", "standing-rule": "Standing rules", "addressed": "Addressed (must not come back)"}
    es = sorted(data["intents"], key=lambda e: (order.get(e["status"], 9), e["id"]))
    cur = None
    for e in es:
        if e["status"] != cur:
            cur = e["status"]
            L += [f"## {heads.get(cur, cur)}", ""]
        L.append(f"### {e['id']} {e.get('subject') or ''}".rstrip())
        L.append("")
        L.append(f"> {e['verbatim']}")
        L.append("")
        bits = [f"said {e['date']}"]
        if e.get("commit"):
            bits.append(f"looking at {e['commit']}")
        if e.get("addressed_in"):
            bits.append(f"addressed in {e['addressed_in']}")
        if e.get("rule"):
            bits.append("rules " + ", ".join(e["rule"]))
        L.append("; ".join(bits) + ".")
        if e.get("note"):
            L.append("")
            L.append(e["note"])
        if e.get("image"):
            L.append("")
            L.append(f"![{e['id']}](images/{os.path.basename(e['image'])})")
        L.append("")
    with open(md, "w") as fh:
        fh.write("\n".join(L))
    return md
