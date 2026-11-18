#!/usr/bin/env python3
"""Extract repo-declared differential-pair skew limits into a --limits JSON for diffpair_skew.py.

Reads source with `ast` only (nothing is imported or executed):
  drone-arm : tools/hardware/audit_rev_b_pcb.py   pair_check(..., name, P, N, limit) in audit_main()
  Code-SDR  : hardware/v2/tools/validate_route.py  MATCHED_PAIRS
              hardware/v2/tools/sensitive_route_contract.py  PAIR_GROUPS (RoutePair / _pair)

    python3 extract_pair_limits.py --drone-arm <repo> --out limits.json
    python3 extract_pair_limits.py --code-sdr <repo> --out limits.json
"""
import argparse, ast, json, os


def lit(node):
    return ast.literal_eval(node)


def drone_arm(repo):
    rel = 'tools/hardware/audit_rev_b_pcb.py'
    tree = ast.parse(open(os.path.join(repo, rel)).read())
    out = []
    for fn in tree.body:
        if isinstance(fn, ast.FunctionDef) and fn.name == 'audit_main':
            for node in ast.walk(fn):
                if isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'pair_check':
                    _, _, name, p, n, lim = node.args
                    out.append(dict(name=lit(name), p=[lit(p)], n=[lit(n)], max_skew_mm=lit(lim),
                                    source=f'{rel} audit_main', require_equal_vias=False))
    return out


def code_sdr(repo):
    out = []
    rel = 'hardware/v2/tools/validate_route.py'
    tree = ast.parse(open(os.path.join(repo, rel)).read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'MATCHED_PAIRS':
            for p, n, lim, desc in lit(node.value):
                out.append(dict(name=desc, p=list(p), n=list(n), max_skew_mm=lim,
                                source=f'{rel} MATCHED_PAIRS', require_equal_vias=True))
    rel = 'hardware/v2/tools/sensitive_route_contract.py'
    tree = ast.parse(open(os.path.join(repo, rel)).read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'PAIR_GROUPS':
            for k, v in zip(node.value.keys, node.value.values):
                fn = v.func.id
                args = [lit(a) for a in v.args]
                if fn == '_pair':
                    p, n, lim = [args[0]], [args[1]], args[2]
                else:
                    p, n, lim = list(args[0]), list(args[1]), args[2]
                out.append(dict(name=lit(k), p=p, n=n, max_skew_mm=lim,
                                source=f'{rel} PAIR_GROUPS', require_equal_vias=True))
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--drone-arm')
    ap.add_argument('--code-sdr')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    pairs = drone_arm(a.drone_arm) if a.drone_arm else code_sdr(a.code_sdr)
    json.dump(dict(pairs=pairs), open(a.out, 'w'), indent=1)
    print(len(pairs), 'limits')
