using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.GameObjects.Systems;

[TestFixture]
public sealed class SpatialTransformTests
{
    [Test]
    public void NestedFramesAndReparentKeepCompleteWorldPose()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var map = sim.CreateMap().MapId;
        var transforms = entities.System<SharedTransformSystem>();
        var parent = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        var target = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        var child = entities.SpawnAttachedTo(null, new EntityCoordinates(parent, Vector2.Zero));
        transforms.SetLocalPose(parent, new RigidTransform3(new Vector3(10, -2, 5),
            Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2)));
        transforms.SetLocalPose(child, new RigidTransform3(new Vector3(2, 0, 3),
            Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 2)));
        transforms.SetLocalPose(target, new RigidTransform3(new Vector3(-100, 40, -9),
            Quaternion.CreateFromYawPitchRoll(0.3f, -1.2f, 0.8f)));
        var world = transforms.GetWorldPose(child);
        Near(world.Position, new Vector3(10, 0, 8));
        Near(world.TransformVector(Vector3.UnitY), Vector3.UnitZ);

        transforms.SetParent(child, target);
        SamePose(transforms.GetWorldPose(child), world);
        Assert.That(entities.GetComponent<TransformComponent>(child).ParentUid, Is.EqualTo(target));
        Assert.That(entities.GetComponent<TransformComponent>(parent).ChildCount, Is.Zero);
        Assert.That(entities.GetComponent<TransformComponent>(target).ChildCount, Is.EqualTo(1));

        var localPoint = new Vector3(4, -3, 2);
        var converted = transforms.ConvertPoint(child, parent, localPoint);
        Near(transforms.GetWorldPose(parent).TransformPoint(converted), world.TransformPoint(localPoint));
    }

    [Test]
    public void PlanarWritesPreserveHeightAndTiltAndMovesReportHeight()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var map = sim.CreateMap().MapId;
        var transforms = entities.System<SharedTransformSystem>();
        var uid = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        var transform = entities.GetComponent<TransformComponent>(uid);
        transforms.SetLocalPose(uid, new RigidTransform3(new Vector3(2, 3, -7.5f),
            Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI / 4)));
        transforms.SetLocalPosition(uid, new Vector2(-10, 40));
        transforms.SetLocalPositionRotation(uid, new Vector2(20, -30), Angle.FromDegrees(90));
        transforms.SetCoordinates(uid, new EntityCoordinates(transform.ParentUid, new Vector2(-5, 6)));
        var pose = transform.LocalPose;
        Near(pose.Position, new Vector3(-5, 6, -7.5f));
        Near(pose.TransformVector(Vector3.UnitZ), new Vector3(MathF.Sqrt(0.5f), 0, MathF.Sqrt(0.5f)));

        MoveEvent? moved = null;
        transforms.OnGlobalMoveEvent += (ref MoveEvent ev) => { if (ev.Sender == uid) moved = ev; };
        transforms.SetLocalPose(uid, new RigidTransform3(new Vector3(-5, 6, 0.25f), pose.Rotation));
        Assert.That(moved.HasValue, Is.True);
        Assert.Multiple(() =>
        {
            Assert.That(moved!.Value.OnlyRotation, Is.False);
            Assert.That(moved.Value.ParentChanged, Is.False);
            Assert.That(moved.Value.OldPose.Position.Z, Is.EqualTo(-7.5f));
            Assert.That(moved.Value.NewPose.Position.Z, Is.EqualTo(0.25f));
            Assert.That(transform.ParentUid, Is.EqualTo(moved.Value.OldPosition.EntityId));
        });
    }

    [Test]
    public void FullAndDeltaCorrectionRestoreHeightOrientationAndNoRotationTransition()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var map = sim.CreateMap().MapId;
        var transforms = entities.System<SharedTransformSystem>();
        var uid = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        var transform = entities.GetComponent<TransformComponent>(uid);
        transforms.SetNoLocalRotation((uid, transform), true);
        var pose = new RigidTransform3(new Vector3(-20, 10, 45), Quaternion.CreateFromYawPitchRoll(0.2f, 0.8f, -0.4f));
        var full = new TransformComponentState(pose.Position, pose.Rotation,
            entities.GetNetEntity(transform.ParentUid), false, false);
        var state = new ComponentHandleState(full, null);
        transforms.OnHandleState(uid, transform, ref state);
        SamePose(transform.LocalPose, pose);
        Assert.That(transform.NoLocalRotation, Is.False);

        var delta = new TransformComponentDeltaState
        {
            ChangedFields = 1UL << SharedTransformSystem.TransformLocalPositionIndex,
            LocalPosition = new Vector3(-20, 10, -0.5f),
        };
        state = new ComponentHandleState(delta, full);
        transforms.OnHandleState(uid, transform, ref state);
        SamePose(transform.LocalPose, new RigidTransform3(delta.LocalPosition, pose.Rotation));
        Assert.That(transform.NextPosition, Is.EqualTo(pose.Position));
        Assert.That(transform.NextRotation, Is.EqualTo(pose.Rotation));
        transforms.SetNoLocalRotation((uid, transform), true);
        Assert.That(transform.LocalPose.Rotation, Is.EqualTo(Quaternion.Identity));
    }

    [Test]
    public void RejectedSpatialReparentDoesNotChangeAnyPoseAxis()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var map = sim.CreateMap().MapId;
        var transforms = entities.System<SharedTransformSystem>();
        var uid = entities.Spawn(null, new MapCoordinates(Vector2.Zero, map));
        var child = entities.SpawnAttachedTo(null, new EntityCoordinates(uid, Vector2.One));
        var pose = new RigidTransform3(new Vector3(-100000, 3, 4000), Quaternion.CreateFromYawPitchRoll(-0.9f, 0.3f, 1.2f));
        transforms.SetLocalPose(uid, pose);
        var transform = entities.GetComponent<TransformComponent>(uid);
        var parent = transform.ParentUid;
        Assert.Throws<InvalidOperationException>(() => transforms.SetParentLocalPose(uid, child, RigidTransform3.Identity));
        Assert.Throws<ArgumentOutOfRangeException>(() => transforms.SetLocalPose(uid, default));
        SamePose(transform.LocalPose, pose);
        Assert.That(transform.ParentUid, Is.EqualTo(parent));
    }

    private static void SamePose(RigidTransform3 actual, RigidTransform3 expected)
    {
        Near(actual.Position, expected.Position);
        foreach (var axis in new[] { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ })
            Near(actual.TransformVector(axis), expected.TransformVector(axis));
    }

    private static void Near(Vector3 actual, Vector3 expected)
        => Assert.That(Vector3.Distance(actual, expected), Is.LessThan(0.0001f), $"Expected {expected}, got {actual}");
}
