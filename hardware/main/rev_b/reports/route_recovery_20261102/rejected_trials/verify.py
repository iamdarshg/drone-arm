#!/usr/bin/env python3
"""Check integrity and recorded DRC counts for archived rejected-route trials."""
import hashlib
import json
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
EXPECTED = {
    "cc1121_rf_via.json": (17, 9, "470c102a5e85a06a8c719811a7d35ac33443022f5f10d2a94ae0c879107b3a54"),
    "cc_rbias_restore.json": (18, 15, "609c9b3bc1e9877a7513f6f1005d0181854552a0c84c4be488cd52794cf870fc"),
    "gnss.json": (16, 38, "bf3661e311c11d741dede702c3b8996e7cdd5ea6a767024bae6d3dc783f3ec70"),
    "hgm_direct_bcu.json": (16, 16, "2ced84f5326de18005fcf03b47a9aad13dfc00c78b30e629e239b27f6e28a4b6"),
    "hgm_escape3.json": (16, 22, "ca899efa88306ceb01107fb32d4b03127b90979f93169bcbc710b3ef6ae64767"),
    "icm.json": (16, 20, "8d414f6d1bee13aa60c34d0f4fe5ab90c1432b9e2438bbb229140736aedf221c"),
    "lna_en_direct_bcu.json": (16, 16, "28e2531bbc8d6140901d2d831ad31738b63bb734e510e8eacf2aea28be6a169b"),
    "lsm_fcu.json": (16, 54, "96669c8ab3ed7237745f57b31484f4f40f0cc533b5009442fdef19e69b6ced87"),
    "pa_en_direct_bcu.json": (16, 19, "76d044bacfdf6e735ec8e055d2854a3d23346ed167611974d47e5ac959f2291f"),
    "rf_reset_restore.json": (20, 32, "f5ddbcb68f2a736c00187d92cd0092aec8d4be4094e2e214355a407ecca85cad"),
    "try1v1v.json": (16, 19, "8478d06831c515052f057607c8156eb7574fa4e48f88d6f8785d4a4fa5f40440"),
    "try3v3out.json": (16, 14, "757e93db508f3fb2bd3c16043568539fdd85e8e43f1be9594560c66c13b9f7f1"),
    "try3v3short.json": (16, 7, "e8b72494e54c656fb9ea9a2c0347347ed4a88b0f7b10a15790b928a468b4a24e"),
    "try_1v1.json": (16, 15, "b4df02e003c94ab58c1199aba6961a61b575f0dc71cef4979839aab9e344da1b"),
    "three_net_3v3_escape.json": (17, 15, "87450fc15782da836c3d7d3ac4b86905abe25c8f9e59e0bb9a55a1c6f415b6fb"),
}

for name, (opens, violations, digest) in EXPECTED.items():
    path = HERE / name
    raw = path.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == digest, f"hash mismatch: {name}"
    report = json.loads(raw)
    assert len(report["unconnected_items"]) == opens, f"open count mismatch: {name}"
    assert len(report["violations"]) == violations, f"violation count mismatch: {name}"
print(f"PASS: {len(EXPECTED)} archived DRC reports match their hashes and recorded counts")
