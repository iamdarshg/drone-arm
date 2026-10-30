"""Seeded pairwise matrix planner; it generates case IDs, never claims execution."""
from __future__ import annotations

import hashlib
import itertools
import json
import random
from typing import Any


def generate_pairwise(axes: dict[str, list[dict[str, Any]]], seed: int = 0, applicability=None) -> dict:
    """Return a deterministic covering set from explicitly supplied source-backed levels.

    Each level needs `{value, unit, source}`. This planner does not assign physical values,
    judge validity, or run a model; every emitted case is therefore `NOT_RUN`.
    """
    if len(axes) < 2: raise ValueError("pairwise planning requires at least two axes")
    names=sorted(axes)
    for axis in names:
        levels=axes[axis]
        if not isinstance(levels,list) or not levels: raise ValueError(f"axis {axis!r} must have sourced levels")
        for i,level in enumerate(levels):
            if not isinstance(level,dict) or not {"value","unit","source"} <= set(level):
                raise ValueError(f"axis {axis!r} level {i} requires value, unit, and source")
            if not level["source"] or level["unit"] is None:
                raise ValueError(f"axis {axis!r} level {i} lacks source or unit")
        signatures=[json.dumps({"value":x["value"],"unit":x["unit"]},sort_keys=True,separators=(",",":")) for x in levels]
        if len(signatures)!=len(set(signatures)): raise ValueError(f"axis {axis!r} repeats a value/unit level")
    combinations=[dict(zip(names,levels)) for levels in itertools.product(*(axes[n] for n in names))]
    rng=random.Random(seed); rng.shuffle(combinations)
    uncovered=set()
    for a,b in itertools.combinations(names,2):
        for va in range(len(axes[a])):
            for vb in range(len(axes[b])): uncovered.add((a,va,b,vb))
    selected=[]
    indexed=[]
    for candidate in combinations:
        idx={n: axes[n].index(candidate[n]) for n in names}
        covered={(a,idx[a],b,idx[b]) for a,b in itertools.combinations(names,2)}
        indexed.append((candidate,idx,covered))
    while uncovered:
        scored=[(len(covered & uncovered),i) for i,(_,_,covered) in enumerate(indexed)]
        best=max(score for score,_ in scored)
        if best == 0: raise RuntimeError("pairwise cover construction stalled")
        i=next(i for score,i in scored if score==best)
        candidate,idx,covered=indexed.pop(i)
        case_values={n: candidate[n] for n in names}
        case_id=hashlib.sha256(json.dumps({"seed":seed,"values":case_values},sort_keys=True,separators=(",",":")).encode()).hexdigest()[:16]
        selected.append({"id":f"SIL-{case_id}","values":case_values,"status":"NOT_RUN","applicability":(applicability or {}).get(case_id,"model_and_bounds_not_validated"),"seed":seed})
        uncovered -= covered
    return {"generator":"seeded_pairwise_v1","seed":seed,"case_count":len(selected),"candidate_count":len(combinations),"pairwise_coverage":"all value pairs for every axis pair", "execution_state":"NOT_RUN", "cases":selected}
