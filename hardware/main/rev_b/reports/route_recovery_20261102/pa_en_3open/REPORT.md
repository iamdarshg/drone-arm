# PA enable route update

This update starts from `6550f35bca6c005773b10eb444e6b270b8690a24`, with source PCB SHA-256 `b107359cf9adf6bb08e9ecd75ea2e0e8e4bddc3036be4e1fd8c3261f36924656`, and KiCad project SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`.

The candidate PCB SHA-256 is `fb2fdc622895e726b569e55fe97b2abd1d14d2c7e81d8e46601e996b89cbd3c3`. Native KiCad DRC reports **3 unconnected items and 0 violations**, down from 4 unconnected items and 0 violations. It closes `/PA_EN`; the remaining opens are `/RF_GPIO2`, `/RF_MISO`, and `/RF_MOSI`. The DRC report SHA-256 is `5020fec7425d9769c1cddef22d2e6eefcfa5884ef352dcd59d5f966c3dbf3e1d`.

The route leaves all 165 footprint references and 657 pad instances, assignments, shapes, sizes, and positions unchanged. It adds an F.Cu-to-In2.Cu 0.3/0.1 mm microvia at `(57.6, 52.2)`, an In2.Cu-to-B.Cu 0.6/0.3 mm blind/buried via at `(58.0, 51.9)`, and copper joining the existing B.Cu path to U41 pad 8. The source-pinned replay applies only the `/PA_EN` copper delta and verifies all target pad and copper geometry.

Run `bash run.sh` with KiCad 9 CLI and the `pcbnew` Python module installed. It retrieves the pinned source board from git, checks source/project/candidate hashes, replays only `/PA_EN`, compares the complete pad and copper geometry, and reruns native DRC on source, candidate, and replay.

This remains **not fabrication-ready**. Three RF nets remain open, and blind/buried microvia capability must be confirmed with the intended board house. RF impedance, signal integrity, assembly yield, electrical behavior, and physical operation have not been tested. No fabrication outputs were generated or approved; no hardware was tested.
