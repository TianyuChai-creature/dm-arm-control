# dm_openarm 0.1.12

CPython 3.13 / Linux x86_64 wheel. Install this wheel in a Python 3.13 environment; it is not compatible with CPython 3.12's binary ABI. Source builds remain available for other supported interpreters.

## Dual-arm automatic zero calibration

- Adds blocking `Arm.calibrate_zero()`, `calibration_status()`, and `cancel_zero_calibration()`, with per-axis reports and `CalibrationError.report` for failures.
- Defaults to firmware MIT kp=120 on CAN 0/1/5/6 and kp=60 on other axes, kd=5, dq=tau=0. Explicit uniform or complete per-axis gains override the defaults. Zero-position correction may increase kp within the documented bounds.
- Scans the two arms in parallel, one joint per arm at a time. Uses stall plus directional torque increase to find endpoints, backs off and repeats, and interpolates the raw encoder zero from measured endpoints. Subsequent angles use native encoder coordinates without scaling.
- Joint/total timing targets and endpoint repeat/span errors are reported rather than used as abort gates.
- Uses station-confirmed motor encoding configuration and fresh disabled angle feedback, without requiring register queries before each run.
- Owns native control exclusively during calibration. Healthy failures latch a hold; feedback faults retain native shutdown handling.
- Sends zero/Flash commands sequentially, without requiring a Flash acknowledgement. Final readback collects new angles from all motors within one second, querying missing axes every 50ms. All angles below 1 degree absolute succeed; an out-of-range angle or an incomplete window fails. Query retries never repeat zero/Flash writes.
- Success clears old commands and leaves motors disabled. A partial write is not atomic and is not automatically retried.

## Verification and scope

Release verification uses a clean Git source archive, a Debug C++ build with assertions, and the installed CPython 3.13 wheel. Exact commands, results, and SHA-256 hashes are recorded in `BUILD_MANIFEST.json`.

On 2026-10-16, the earlier single-query implementation completed all ten axes' zero/save commands. Independent readback confirmed all ten at approximately -0.000191 rad and disabled. That run's API reported missing final feedback; the final one-second readback window is covered by offline tests and has not been rerun on hardware. Power-cycle retention remains unverified. See `dm_openarm/hardware_verification_20261016.json`.

The configured USB serial must match the user's adapter. This release does not change consumer projects or operate hardware during packaging.
