using System.Numerics;
using NUnit.Framework;
using Robust.Shared.ContentPack;
using Robust.Shared.EntitySerialization;
using Robust.Shared.EntitySerialization.Systems;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.Shared.Serialization;
using Robust.Shared.Utility;

namespace Robust.UnitTesting.Shared.EntitySerialization;

[TestFixture]
public sealed class SpatialTransformPersistenceTest : RobustIntegrationTest
{
    [Test]
    public async Task MapRoundTripRemapsParentsWithoutLosingHeightOrTilt()
    {
        using var server = StartServer(new ServerIntegrationOptions { Pool = false });
        await server.WaitIdleAsync();
        await server.WaitAssertion(() =>
        {
            var entities = server.EntMan;
            var maps = server.System<SharedMapSystem>();
            var transforms = server.System<SharedTransformSystem>();
            var loader = server.System<MapLoaderSystem>();
            var path = new ResPath("/spatial-transform-roundtrip.yml");
            maps.CreateMap(out var mapId);
            var parent = entities.Spawn(null, new MapCoordinates(Vector2.Zero, mapId));
            var child = entities.SpawnAttachedTo(null, new EntityCoordinates(parent, Vector2.Zero));
            var parentPose = new RigidTransform3(new Vector3(-20, 30, 10), Quaternion.CreateFromYawPitchRoll(0.2f, 0.8f, -0.1f));
            var childPose = new RigidTransform3(new Vector3(0.5f, -0.5f, -4), Quaternion.CreateFromYawPitchRoll(-0.4f, 0.1f, 0.7f));
            transforms.SetLocalPose(parent, parentPose);
            transforms.SetLocalPose(child, childPose);
            var world = transforms.GetWorldPose(child);
            Assert.That(loader.TrySaveMap(mapId, path), Is.True);
            using (var reader = server.Resolve<IResourceManager>().UserData.OpenText(path))
            {
                var text = reader.ReadToEnd();
                Assert.That(text, Does.Contain("format: 8"));
                Assert.That(text, Does.Contain("transformSchema: 1"));
            }
            maps.DeleteMap(mapId);

            Assert.That(loader.TryLoadMap(path, out var firstMap, out _), Is.True);
            Assert.That(loader.TryLoadMap(path, out var secondMap, out _), Is.True);
            var firstParent = OnlyChild(entities, firstMap!.Value.Owner);
            var secondParent = OnlyChild(entities, secondMap!.Value.Owner);
            var firstChild = OnlyChild(entities, firstParent);
            var secondChild = OnlyChild(entities, secondParent);
            Assert.That(firstParent, Is.Not.EqualTo(secondParent));
            Assert.That(firstChild, Is.Not.EqualTo(secondChild));
            foreach (var (newParent, newChild) in new[] { (firstParent, firstChild), (secondParent, secondChild) })
            {
                SamePose(entities.GetComponent<TransformComponent>(newParent).LocalPose, parentPose);
                SamePose(entities.GetComponent<TransformComponent>(newChild).LocalPose, childPose);
                SamePose(transforms.GetWorldPose(newChild), world);
                Assert.That(entities.GetComponent<TransformComponent>(newChild).ParentUid, Is.EqualTo(newParent));
            }
        });
    }

    [Test]
    public async Task WireSnapshotsAndDeltasPreserveCanonicalPose()
    {
        using var server = StartServer(new ServerIntegrationOptions { Pool = false });
        await server.WaitIdleAsync();
        await server.WaitAssertion(() =>
        {
            var serializer = server.Resolve<IRobustSerializer>();
            var pose = new RigidTransform3(new Vector3(12, -100000, -4.25f),
                Quaternion.CreateFromYawPitchRoll(-0.8f, 0.3f, 1.4f));
            var original = new TransformComponentState(pose.Position, pose.Rotation, default, false, false);
            using var stream = new MemoryStream();
            serializer.Serialize(stream, original);
            stream.Position = 0;
            var restored = serializer.Deserialize<TransformComponentState>(stream);
            Assert.That(restored.LocalPosition, Is.EqualTo(pose.Position));
            Assert.That(restored.Rotation, Is.EqualTo(pose.Rotation));

            var delta = new TransformComponentDeltaState
            {
                ChangedFields = (1UL << SharedTransformSystem.TransformLocalPositionIndex)
                    | (1UL << SharedTransformSystem.TransformLocalRotationIndex),
                LocalPosition = new Vector3(-9, 3, 17),
                Rotation = Quaternion.Identity,
            };
            stream.SetLength(0);
            stream.Position = 0;
            serializer.Serialize(stream, delta);
            stream.Position = 0;
            var restoredDelta = serializer.Deserialize<TransformComponentDeltaState>(stream);
            restoredDelta.ApplyToFullState(restored);
            Assert.That(restored.LocalPosition, Is.EqualTo(delta.LocalPosition));
            Assert.That(restored.Rotation, Is.EqualTo(Quaternion.Identity));
            Assert.That(restored.ParentID, Is.EqualTo(original.ParentID));
            Assert.That(original.LocalPosition, Is.EqualTo(pose.Position));
        });
    }

    private static EntityUid OnlyChild(IEntityManager entities, EntityUid parent)
    {
        using var children = entities.GetComponent<TransformComponent>(parent).ChildEnumerator;
        Assert.That(children.MoveNext(out var child), Is.True);
        Assert.That(children.MoveNext(out _), Is.False);
        return child;
    }

    private static void SamePose(RigidTransform3 actual, RigidTransform3 expected)
    {
        Assert.That(Vector3.Distance(actual.Position, expected.Position), Is.LessThan(0.0001f));
        foreach (var axis in new[] { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ })
            Assert.That(Vector3.Distance(actual.TransformVector(axis), expected.TransformVector(axis)), Is.LessThan(0.0001f));
    }
}
