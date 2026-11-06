# Spatial backend qualification (E00 / G0)

Status: in progress. These checks qualify a dependency; they do not implement E03
or claim the complete spatial platform works.

Baseline upstream: `4058566ed460e156d77d32b1810fd97ba5a57ff9` (295.0.1).
Fork baseline: `cbfc5a0f632a68d74c25eec6949be11a0f36091e` (identical tree).
SDK: 10.0.100. Candidate backend: BepuPhysics 2.4.0, pinned centrally and
referenced by Robust.Shared with private compile assets; qualification code stays
in the existing engine test project. Shipping the dependency does not yet add the
E03 ECS backend.
No Content assembly receives BEPU handles or a direct dependency.

Run from the engine root:

```sh
dotnet restore
dotnet build -m:2 --no-restore
dotnet test --no-build --no-restore
dotnet test Robust.Shared.Tests/Robust.Shared.Tests.csproj \
  --filter FullyQualifiedName~BepuQualificationTest
```

The fixture checks +Z-up gravity and support at fixed 30/60Hz, body removal,
raycasts distinguishing vertically stacked surfaces, height-aware shape sweeps,
a rotating bar sweep that hits between its clear endpoint poses, and a 300m/s sphere against
a 0.1m wall. Tests run without GPU or game content. They are not benchmarks.
The constraint case suspends a dynamic body from a kinematic anchor, verifies
the retained height, removes the anchor and verifies the released body falls.
Compound raycasts distinguish solid children from their gap; an initially
overlapping shape reports a zero-time hit. A dynamic spinning box also verifies
its full oriented extent stays outside a wall at both 30 and 60Hz.
All thirteen physics cases passed on the Linux
x64 .NET 10.0.100 functional runner.

Two additional cases compile actual content-shaped assemblies and run the normal
IL/type verifier: a public Robust call is accepted; a direct BEPU call is rejected
specifically for its backend reference. The sandbox whitelist is unchanged.

CCD qualification exposed two independent settings that must be recorded with
the eventual physics profile: temporal sweep tolerance versus speed/thickness,
and contact stiffness versus soft-contact penetration. BEPU's default 1ms tolerance
spans 0.3m at 300m/s. A 30Hz contact spring permitted penetration even with eight
solver substeps. The passing dynamic fixture uses 1us sweep tolerance, a 1m
maximum speculative margin, a 240Hz critically damped contact spring, eight
velocity iterations and one solver substep. This is a
candidate test profile, not an unmeasured production performance recommendation.
Do not weaken the collision assertion to qualify a failing profile.

Reference for those semantics:
https://github.com/bepu/bepuphysics2/blob/v2.4.0/Documentation/ContinuousCollisionDetection.md

`Tools/package_standalone_client.py` publishes a Release engine client with its
.NET runtime and native libraries, records the engine SHA, source cleanliness,
SDK, BEPU version and payload hashes, and includes the engine resources. Content
owns combining it with its matching content archive. Unlike launcher packages,
it needs no separately provisioned launcher native library bundle. It refuses to
overwrite an existing artifact. Run it after tests because publish shares
intermediate outputs, although this tool does not delete `bin`.

The spatial qualification workflow runs the complete engine suite followed by
standalone packaging on Ubuntu 24.04 and Windows Server 2022, using SDK 10.0.100.
It records the actual hosted image version and uploads evidence. A workflow file
is not evidence that either remote job has passed.

Still required before E00 is closed: loading the updated packaged dependency
through the trusted Robust path with content sandbox enabled, supported platform
runs, allocation/performance budgets, GPU reference profile and joint acceptance.
Cross-platform bitwise determinism is not assumed.

User's target RAM is 8GB. The current Linux functional runner has a 32GiB cgroup
limit, four CPU quota cores and software Mesa llvmpipe. Its results do not certify
the final 8GB client/server SLO or hardware-accelerated rendering performance.
