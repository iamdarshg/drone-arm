# FreeRouting 2.5.0 trial — rejected

The autorouter was run against the current promoted board (SHA-256 `60da183ad175be4e1db9daa57a705714a84dd0d7b1f0b2a942f6beeee3a9a191`) and matching project (SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`). The input DSN and output SES are preserved here.

FreeRouting 2.5.0 JVM build ran with Temurin JRE 25.0.5, `-Xms128m -Xmx700m -Xss512k -XX:+UseSerialGC -XX:ActiveProcessorCount=2 -XX:ReservedCodeCacheSize=64m`, and `-mp 10 -mt 1 -us Global -is prioritized`. It completed ten global passes after 44m 32.91s. Fanout escaped 494 of 591 SMD pins (83.6%). The router reported 34 unrouted items after fanout; pass 1 briefly reached 32, and passes 2–10 stayed at 34 with the same score and six fixed-item conflicts. It did not run its optimization stage because unrouted connections remained.

After SES import and zone refill, KiCad 9.0.2 DRC reported 16 unconnected items and 12 violations: the same 4 J1 hole-clearance findings plus 5 dangling tracks and 3 dangling vias. Since the candidate baseline is 16 unconnected / 4 violations, the autorouter session is rejected and not applied to `main`.

The first attempt with the native CLI binary crashed with a GraalVM serialization `UnsupportedFeatureError` while snapshotting the board. The JVM build ran to completion; the full logs for both attempts are retained as `native-cli-crash.log` and `freerouting-jvm.log`. FreeRouting 2.5.0 Linux x64 CLI archive SHA-256: `64a957a9ef2a2fbde6ab2001bb32800f5ba34f9fd15575e363512934b870d63a`. JAR SHA-256: `f6f51bb02245e8e717f9359bd260cc9c5c0b1bc0acc8b7cb2cd5b8ffeb5de3c7`.

To independently reproduce SES import and the DRC finding from the repository root, run:

```sh
python3 hardware/main/rev_b/reports/freerouting_20261102/verify.py
```

This requires KiCad CLI 9.0.2 and the system `/usr/bin/python3` with KiCad `pcbnew` 9.0.2. The verifier imports the pinned SES into a temporary board copy, refills zones, reruns DRC with the matching project, and compares violation signatures with `imported_drc.json`.
