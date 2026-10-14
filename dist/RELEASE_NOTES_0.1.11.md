# dm_openarm 0.1.11

0.1.11 consolidates the real-Teleop 0.1.7.post2 contract on the 0.1.10 safety implementation. The historical post1/post2 changes are already present; no older implementation is reapplied over the newer checks.

## Retained post2 capabilities

- Shared USB transport, disjoint per-arm motor ownership, and serialized CAN sends.
- `Arm.from_yaml(side=...)` retains the full bus registration while selecting one arm.
- Passive connect and read-only 0xCC status probes without motor enable/disable frames.
- Status probes and register replies are isolated from motion feedback.
- Seeded startup, measured-position hold, stationary-command hold, and command/feedback timeout protection.

## Retained 0.1.10 protections

USB serial verification; fresh enabled feedback; complete MIT batch prevalidation; native partial command merging; synchronous hold-batch submission; confirmed disable; retained ownership on unknown state; and maintenance exclusion while the MIT loop runs. Timing diagnostics remain available.

The adapter runtime is unchanged. This is a versioned source build, not a rewrite of an old wheel. Source, toolchain, hashes and offline verification results are recorded in BUILD_MANIFEST.json.

## Consumer migration

real-Teleop is pinned to the new wheel and hash and its environment is synchronized. Shutdown failures must retain the SDK handle and report the original fault. Persistent zeroing still requires a stopped loop and fresh confirmed motor state; a firmware that stops feedback can cause a safe refusal. No safety check is bypassed for legacy compatibility.

## Validation scope

Native fake-transport and installed-wheel API checks cover the retained contract. Physical hold stability, feedback timing after disable, bus load, emergency stop and Flash-zero calibration require separate hardware acceptance; no motor is actuated during this release workflow.
