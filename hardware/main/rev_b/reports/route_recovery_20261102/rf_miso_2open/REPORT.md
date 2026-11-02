# RF MISO route update

This route starts from commit `01e83dab330b11c4914380c022e3ec2d92ed492b`, PCB SHA-256 `fb2fdc622895e726b569e55fe97b2abd1d14d2c7e81d8e46601e996b89cbd3c3`, with project SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`.

The candidate PCB SHA-256 is `fde09ade461c58ad4b3cc1952dca15e961e417dea0e5875055b503114e83be3a`. Fresh native KiCad DRC reports **2 unconnected items and 0 violations**, down from 3/0; `/RF_MISO` is closed and the remaining opens are `/RF_GPIO2` and `/RF_MOSI`. The archived DRC report SHA-256 is `fd30a5e507f9cee8b2b9691af36e354d47b50840035ace5ae47eeaae81397fac`.

All 165 footprint references and 657 physical pad instances, including net assignments, shapes, sizes, and positions, are unchanged. The copper delta affects `/RF_MISO` and `/RF_SCLK`. It adds a stacked 0.3/0.1 mm F.Cu-to-In1.Cu and In1.Cu-to-In2.Cu microvia pair at `(53.9, 53.0)`, routes on In2.Cu to a 0.6/0.3 mm In2.Cu-to-B.Cu blind/buried transition at `(56.0, 52.5)`, follows B.Cu to a 0.6/0.3 mm F.Cu-to-B.Cu through-via at `(92.0, 63.4)`, then reaches U40 pad 9. A local `/RF_SCLK` In2.Cu detour clears the source transition and preserves that net.

Run `bash run.sh` with KiCad 9 CLI and `pcbnew` Python installed. It retrieves the pinned source from git, verifies source/project/candidate hashes, replays only the `/RF_MISO` and `/RF_SCLK` copper, compares all pad and copper geometry, and reruns native DRC on source, target, and replay.

This board is **not fabrication-ready**. Two RF nets remain open. The route uses stacked microvias and a blind/buried via; the intended board house's capability for those structures is unknown. RF impedance and signal integrity have not been simulated, and there has been no physical assembly or test. No fabrication files have been generated or approved.
