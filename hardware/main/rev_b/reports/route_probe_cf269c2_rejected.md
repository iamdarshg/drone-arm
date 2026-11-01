# Rejected Rev-B route probes from `cf269c2`

Both disposable candidates were derived from source commit `cf269c20a33ddb44010b9d2066227a42aa537aed`, with board SHA-256 `79ad895d0c525f15e949773f79808c77164fd09495c8d576f6b80390bee4baee` and matching project SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`. For each candidate and baseline, KiCad pcbnew 9.0.2 explicitly refilled zones with `ZONE_FILLER.Fill(board.Zones())`; KiCad CLI 9.0.2 then ran `pcb drc --severity-all --format json` with a same-stem copy of the matching project. Fresh zone-refilled baseline was 7 violations / 44 unconnected items in both trials: LSM_INT1 baseline JSON SHA-256 `2e8dd2cb61be745f78c9d975b6e6eb1f0ad6376dd0f8f3bb5005f04d94311fc2`; GND baseline JSON SHA-256 `54bbd072c421182587e3ff9f261b65dd5b0d48b0b863c554724705de6111e76a`.

## `/LSM_INT1` via escape and In2.Cu span

The disposable route escaped U10.4 `(54.05,47.4)` to a 0.6/0.3 mm through-via at `(53,47.4)`, crossed In2.Cu to a second through-via at `(59.4,83.6)`, and joined the existing F.Cu `/LSM_INT1` stub ending at `(58.695,83.8925)`. Candidate board SHA-256: `7bff724d17e8a6350cedad471408ce5ccf488b95ed2c9e0f05c99da641b2de49`. Candidate DRC JSON: [`route_probe_cf269c2_lsm_int1_candidate.json`](route_probe_cf269c2_lsm_int1_candidate.json), SHA-256 `9614ff4835ea14b09a0fa6f955f2094deaf5cba136f1beecb1fe47a76518f0a9`; result **26 violations / 43 unconnected**. The route reduces one open but adds 19 violation signatures, including shorts/crossings with `/BARO_SCL` and `/SENS_MOSI` plus via clearances. Rejected; no candidate board was promoted.

## `/GND` plane tie via

A 0.6/0.3 mm through-via at `(35,90)` was tested as a second ground-plane stitch location. Candidate board SHA-256: `15b3a4a4560b2d001de9c60cc08a5f0c37b793798768294bd6a88bf58e881b9f`. Candidate DRC JSON: [`route_probe_cf269c2_gnd_tie_candidate.json`](route_probe_cf269c2_gnd_tie_candidate.json), SHA-256 `728abc48b0012b9a20bf76aa9e56c4fe5318667bb3a2a7e55b30c4bedd2a0af1`.

The run reports **7 violations / 44 unconnected**, with all seven signatures unchanged, so this via did not reduce the unrouted count. Rejected; no candidate board was promoted.
