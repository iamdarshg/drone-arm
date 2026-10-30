# Independent acceptance review

The independent verifier ran in clean sibling Git worktrees using the pinned acceptance environment (`CadQuery 2.5.2`, `OCP 7.7.2`, Python 3.12.14). No physical equipment or paid compute was used. The worktrees were clean at the start and end of verification.

The final source revisions were:

- `drone-arm`: `f8b525d329a252b22f815d3f5fe5f0327b2d577a` for the full `all` profile.
- `drone-arm`: `00158440550ee3d997fe8df60d33b0966abf6339` for the later `evidence` profile and final test suite. This revision adds the locked evidence profile and CI mapping; it does not change the physical gate definitions used in the full profile.
- `drone-mech`: `fd8decc7c6683692ab04150a06c201ee7138aa3a` for both runs.

The system configuration SHA-256 was `0112688d9b685e7c983d491aba0fd94e0fee8c997eccd4690e690dfe46601b0c`; the all-profile test-definition SHA-256 was `9baf04ed130248d68c4c3c5c88f537ce0db8cc69c1051581aff02abec66cfc23`.

`python -m engineering_acceptance validate-inputs` exited 0. The combined acceptance and independent suites (`python -m pytest -q tests/acceptance tests/independent`) passed 84 tests. The native mechanical mutation command (`python -m highspeed_edf.verification.negative_fixtures`) detected all 14 injected cases and exited 0. The independent suite includes a temporary Git worktree mutation: pinned source bytes pass unchanged and fail after a file is altered without moving HEAD. It also verifies forged or empty FSI receipts, raw evidence tampering, nonfinite and dimensionally invalid values, stale PCB hashes, missing payload mass, and negative solver claims.

The full `all` profile attempted 25 checks and exited 1, with 9 PASS, 9 FAIL, and 7 BLOCKED. The runner manifest reports `integrity_ok: true`; both repository HEADs stayed fixed for the run. The nine failures are expected baseline findings: the selected ESC board exceeds the CAD placeholder; six motor pairs intersect; the fuselage battery-bay shell misses its reserved envelope; seven effectors exceed six outputs; the safe-start gate and firmware source/build gates fail because required firmware sources are absent; KiCad reports 600 DRC/open-net findings; and 22 FSI receipts record failed, timed-out, or unconverged attempts. The seven BLOCKED gates concern connector service access, mass overrides in the native CG calculation, absent physical release evidence, missing converged aero evidence, missing payload geometry/electronics evidence, absent temperature-sensor measurements, and an unresolved historical cost/teardown receipt.

The `evidence` profile on arm `0015844` attempted 8 checks and exited 1: 3 PASS, 1 FAIL, and 4 BLOCKED. Its one failure is the same set of 22 failed or unconverged FSI receipts. The negative aero status fixtures are now treated as nonclaims; absent qualified solver evidence remains BLOCKED. `python -m engineering_acceptance verify-evidence` exited 0 against the generated ledger and manifests.

The all-profile run ID is `09eea1205abe215ed14eecb5129f552415276386e12860ac705193beb086758f`, manifest SHA-256 `f6f9e4e60f4bb7db770580a1806c1c0be247a2c10680661de7e8147bb8fb2379`. The evidence-profile run ID is `41cd0a8c8a424d40cff0d0ac76eb216c3d774caf2ae1b12eadd18a9c6c7df36d`, manifest SHA-256 `ae2194767eef66d4930e04a73d298ac05b8a790929b73f6aab5f22bb05c51a3c`.

Generated scorecards and the native fixture receipt are in `/workspace/final-verification/artifacts/`:

| Artifact | SHA-256 |
|---|---|
| `all-scorecard.json` | `0be90e9576b066e3aaa4687773fedf2b5b5dd5cebc4d23e85e7f01909a0a324a` |
| `all-scorecard.xml` | `b37f315f331c18cc66f67357ed0d6e18d85f8f6c46466f4a8df85a41eef51e76` |
| `all-scorecard.html` | `73192f225cc5ecb1882f6242e2fa00c9dce8d0323d3cde552b1100fd2fecf044` |
| `evidence-scorecard.json` | `4719e984cc4d830cc7aad6e58b4cd64c2aa40b8144d8184bd8b2396e96b44733` |
| `evidence-scorecard.xml` | `e1636f2ecb26bcf47ac1f3351fad4bf7924d0143cf0a53fb8cb7c2bca3953a67` |
| `evidence-scorecard.html` | `ad3500319e1c63170826dac6b6552202337552744397a9f225329f54a817e1bc` |
| `native14.json` | `40db591f66aa6f84d4024ff48bc9cdfee648ca1e76650e5a3717c1af838310cd` |

No aircraft release, fabrication, or flight-readiness claim is supported by these software results.
