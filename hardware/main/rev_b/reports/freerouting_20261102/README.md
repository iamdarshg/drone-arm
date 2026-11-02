# FreeRouting 2.5.0 trial — rejected

The autorouter was run against the historical 16-open board (SHA-256 `60da183ad175be4e1db9daa57a705714a84dd0d7b1f0b2a942f6beeee3a9a191`) pinned at Git revision `7f9a9eff504ad5ebc7f33bd1af25ee743d9381c1` and the matching project (SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`). The candidate has since been improved to 15 opens; the input DSN and output SES remain an archived diagnostic trial.

FreeRouting 2.5.0 JVM build ran with Temurin JRE 25.0.5, `-Xms128m -Xmx700m -Xss512k -XX:+UseSerialGC -XX:ActiveProcessorCount=2 -XX:ReservedCodeCacheSize=64m`, and `-mp 10 -mt 1 -us Global -is prioritized`. It completed ten global passes after 44m 32.91s. Fanout escaped 494 of 591 SMD pins (83.6%). The router reported 34 unrouted items after fanout; pass 1 briefly reached 32, and passes 2–10 stayed at 34 with the same score and six fixed-item conflicts. It did not run its optimization stage because unrouted connections remained.

After SES import and zone refill, KiCad 9.0.2 DRC reported 16 unconnected items and 12 violations: the same 4 J1 hole-clearance findings plus 5 dangling tracks and 3 dangling vias. Against its pinned 16-unconnected / 4-violation input, the autorouter session is rejected and was not applied to `main`.

A second run used `-mp 5 -mt 1 -us Greedy -is random`. It also completed five passes with the same 34 unrouted items and six fixed conflicts. It produced the identical SES SHA-256 (`f9ba660a4c6f1e9a91017830b3c529d4e0b98fc4ed004dcf6a2403dc01b3bde6`) and the same imported DRC result (16 unconnected / 12 violations). Its log is `freerouting-greedy.log`.

The first attempt with the native CLI binary crashed with a GraalVM serialization `UnsupportedFeatureError` while snapshotting the board. The JVM build ran to completion; the full logs for both attempts are retained as `native-cli-crash.log` and `freerouting-jvm.log`. FreeRouting 2.5.0 Linux x64 CLI archive SHA-256: `64a957a9ef2a2fbde6ab2001bb32800f5ba34f9fd15575e363512934b870d63a`. JAR SHA-256: `f6f51bb02245e8e717f9359bd260cc9c5c0b1bc0acc8b7cb2cd5b8ffeb5de3c7`.

To independently reproduce SES import and the DRC finding from the repository root, run:

```sh
python3 hardware/main/rev_b/reports/freerouting_20261102/verify.py
```

This requires KiCad CLI 9.0.2 and the system `/usr/bin/python3` with KiCad `pcbnew` 9.0.2. The verifier imports the pinned SES into a temporary board copy, refills zones, reruns DRC with the matching project, and compares violation signatures with `imported_drc.json`.
