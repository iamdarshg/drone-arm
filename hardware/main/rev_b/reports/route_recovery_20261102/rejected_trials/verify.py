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
    "one_v_one_direct.json": (15, 6, "5a509eda84f8b80ded61bb0fd090aac4d50d9e01b646a8eba5770ae8e354e674"),
    "one_v_one_bcu.json": (15, 8, "10cd993aea4772282bd3127d516cc10857608f34e7c074a44947962e965a5915"),
    "one_v_one_bcu_detour.json": (15, 11, "7a3866f85bc803c89739173604f56cef0978a5b0d2c6784ab7ad62c4a30d6955"),
    "one_v_one_in1.json": (15, 7, "ca759882314b3a0907e94272e2200152f904f8fb390a05fdb242de04d3782503"),
    "rf_spi_bundle.json": (13, 79, "058c3f65349d9306daa7173ab4d1b21440b3f86e95a17c45c9d71ff6b5ad571f"),
    "usb_dp_via.json": (15, 12, "a04cb3bcd615d0c067543fbe411355531f1b7b2ea5312fe4b86bbaa2f161645d"),
    "usb_dp_direct.json": (15, 11, "45af00464ddb0a7af1cf4555cda809364fa0ebb8c3cb925dbc6f6ac540e1d48c"),
    "three_net_3v3_pad29.json": (15, 11, "1ceae46b998f68389477c983544ce48693291cc6d703b3efb0ed963099e439d6"),
    "gnss_rx_via_and_pps_relocate.json": (18, 29, "3a0a1c9bf3ddfa12ab433b1be71be860575fcebc4c9c6211d8b443ce3bb2ab24"),
    "cc_rbias_direct_probe.json": (15, 8, "1b12d6376cc73ca6578420b11e7f050f061a7a9609295c6c6b0d07ecddcfe044"),
    "cc_rbias_pad15_via_y64_0.json": (16, 8, "96085a6a4a3b8e1f56e62ae522eafc9e7fed78366d48c684f653d9857744d398"),
    "cc_rbias_pad15_via_y64_1.json": (16, 7, "a041cffb3f87dd8471d58fc4fd075e136e1d9fb3e6a96d8bcb165a9796dec356"),
    "cc_rbias_pad15_via_y64_3.json": (16, 9, "d807a8255d2ff975df44b21b6c743b8af50cb93653901813af5e75df4a58e435"),
    "cc_rbias_pad15_east_via.json": (16, 11, "8f24678d149763c9df453bbc302c673b19ed017ee429ed82b13f7b7df4dcd865"),
    "cc_lna_p_direct.json": (15, 9, "349859749061ccc2199dc695cce300f0caaad1fb89e941582d34cb551b871720"),
    "cc_lna_p_detour_y0.json": (15, 28, "017e308738ec2d5fb2880b389d4b00a241e21c1381c840b0cfc59919ac11431f"),
    "cc_lna_p_detour_y1.json": (15, 19, "42889db7c6cae41c9a1ec1575ce4d83b1c43ce53fc46d3aab70550abb1f65b15"),
    "cc_lna_p_detour_y2.json": (15, 29, "ead92049fc1d53f11338ccd4923b457e8d1cef2d4c25964b62d1ade7f9b46e9b"),
    "cc_lna_p_detour_y3.json": (15, 33, "d5661d3ed14a0a9e64bbc014efa5c4711459b49afce92f7affd665671b3ba490"),
    "cc_lna_p_detour_y4.json": (15, 26, "b7659539f189db81a3f5312928c653560c45944e9cec1d69730baf53f972eaf6"),
    "cc_lna_p_via_pair.json": (15, 26, "bd4565047dae323cea69df40769d11efd5f86deafacda5919c09b75f09d988d2"),
    "cc1121_rf_direct.json": (15, 10, "06f66f789a159af3786c32f72b440f8c1ef2b7b4b1397fc7720b368ae72c10df"),
    "mcu_three_net_breakout.json": (13, 48, "cff09bf74e1711ba1b503040da64fcb3db639a19b9a423d2016276efbf048580"),
    "pa_en_via_bcu.json": (15, 18, "62ba7d71149e84245ce8b86b9aaa0a6303034ad81f4edcd08cc19579a126c3c0"),
    "icm_int1_detour.json": (15, 12, "e98ee782f2d71280fb759ba3fd802a62436d8a857a81e4a3a71228732a6fbfb3"),
    "sensor_interrupt_pair_in2_bcu.json": (14, 37, "cb951bd6cf245bc05fb64f87eed851e65b4d5ac6f1d092fad05439578eabcad0"),
    "gnss_rx_in2_keep_pps.json": (15, 41, "eec38ae7dfcd866e3898abc3bd2d96157ea78edc94c5e3b6e20dcce0cb75704a"),
    "gnss_rx_in1_keep_pps.json": (15, 48, "4583043194a5724547e49dbffaeb99d62456d5a32084b0574b563a5aa9feb8d8"),
    "rf_reset_bcu_local_escape.json": (15, 13, "a30cd7cc899b6b2ad3b488f89d1989a6bbca6571edc8f498e04614f38a42fa06"),
    "usb_dp_microvia_pair.json": (15, 15, "1ae83869bcc9981139e58405364e5d70e6ec8d8f4c27c5864b2ea747df98c7d6"),
    "cc1121_rf_35_bcu.json": (15, 8, "b6c7fd929e07b60f966391ea6ce724e88245ddae507f4155d569c358f9c07428"),
}

for name, (opens, violations, digest) in EXPECTED.items():
    path = HERE / name
    raw = path.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == digest, f"hash mismatch: {name}"
    report = json.loads(raw)
    assert len(report["unconnected_items"]) == opens, f"open count mismatch: {name}"
    assert len(report["violations"]) == violations, f"violation count mismatch: {name}"
print(f"PASS: {len(EXPECTED)} archived DRC reports match their hashes and recorded counts")
