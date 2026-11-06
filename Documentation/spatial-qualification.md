# Spatial backend qualification (E00 / G0)

Status: in progress. These checks qualify a dependency; they do not implement E03
or claim the complete spatial platform works.

Baseline upstream: `4058566ed460e156d77d32b1810fd97ba5a57ff9` (295.0.1).
Fork baseline: `cbfc5a0f632a68d74c25eec6949be11a0f36091e` (identical tree).
SDK: 10.0.100. Candidate backend: BepuPhysics 2.4.0, pinned centrally and
referenced only from the existing engine test project during qualification.
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

Still required before E00 is closed: overlap and rotating dynamic-body CCD,
compound/constraint cases, loading through the trusted Robust engine path with
content sandbox enabled, supported platform runs, allocation/performance
budgets, GPU reference profile and the joint packaging/client acceptance.
Cross-platform bitwise determinism is not assumed.

User's target RAM is 8GB. The current Linux functional runner has a 32GiB cgroup
limit, four CPU quota cores and software Mesa llvmpipe. Its results do not certify
the final 8GB client/server SLO or hardware-accelerated rendering performance.
