using NUnit.Framework;
using Robust.Shared.ContentPack;
using Robust.Shared.Configuration;
using Robust.Shared.EntitySerialization.Systems;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Log;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.Spatial;
using Robust.Shared.Utility;

namespace Robust.UnitTesting.Shared.EntitySerialization;

[RegisterComponent]
internal sealed partial class BoundaryKeySaveTestComponent : Component
{
    [DataField] public BoundaryKey Key;
}

[TestFixture]
public sealed class BoundaryKeyPersistenceTest : RobustIntegrationTest
{
    [Test]
    public async Task TwoPrefabCopiesRemapFrameQualifiedBoundaryAndLayerIdentity()
    {
        using var server = StartServer(new ServerIntegrationOptions { Pool = false });
        await server.WaitIdleAsync();
        await server.WaitAssertion(() =>
        {
            var entities = server.EntMan;
            var loader = server.System<MapLoaderSystem>();
            var map = server.System<SharedMapSystem>().CreateMap();
            var frame = entities.SpawnAttachedTo(null, map, RigidTransform3.Identity);
            var segment = entities.SpawnAttachedTo(null, frame, RigidTransform3.Identity);
            var key = new BoundaryKey(frame, new LayerId(29), -7, 4, BoundaryAxis.Vertical);
            entities.EnsureComponent<BoundaryKeySaveTestComponent>(segment).Key = key;
            var path = new ResPath("/boundary-prefab.yml");
            Assert.That(loader.TrySaveEntity(frame, path), Is.True);
            entities.DeleteEntity(frame);

            Assert.That(loader.TryLoadEntity(path, out var first), Is.True);
            Assert.That(loader.TryLoadEntity(path, out var second), Is.True);
            Assert.That(first, Is.Not.EqualTo(second));
            var keys = new HashSet<BoundaryKey>();
            foreach (var loaded in new[] { first!.Value, second!.Value })
            {
                using var children = entities.GetComponent<TransformComponent>(loaded).ChildEnumerator;
                Assert.That(children.MoveNext(out var child), Is.True);
                Assert.That(children.MoveNext(out _), Is.False);
                var restored = entities.GetComponent<BoundaryKeySaveTestComponent>(child).Key;
                restored.Validate();
                Assert.That(restored.OwnerFrame, Is.EqualTo(loaded.Owner));
                Assert.That(restored.OwnerFrame, Is.Not.EqualTo(frame));
                Assert.That(restored.LayerId, Is.EqualTo(key.LayerId));
                Assert.That(restored.Endpoints(), Is.EqualTo(key.Endpoints()));
                keys.Add(restored);
            }
            Assert.That(keys, Has.Count.EqualTo(2));

            string saved;
            using (var reader = server.Resolve<IResourceManager>().UserData.OpenText(path))
                saved = reader.ReadToEnd();
            Assert.That(saved, Does.Contain("y: 4"));
            var beforeInvalidLoad = entities.EntityCount;
            var config = server.Resolve<IConfigurationManager>();
            var logFailureLevel = config.GetCVar(RTCVars.FailureLogLevel);
            try
            {
                // MapLoader logs then rethrows this expected validation failure.
                // Allow that diagnostic so its normal cleanup can execute.
                config.SetCVar(RTCVars.FailureLogLevel, LogLevel.Fatal);
                using var invalid = new StringReader(saved.Replace("y: 4", "y: 2147483647"));
                Assert.Throws<ArgumentOutOfRangeException>(() =>
                    loader.TryLoadEntity(invalid, "invalid-boundary-endpoint", out _));
            }
            finally
            {
                config.SetCVar(RTCVars.FailureLogLevel, logFailureLevel);
            }
            Assert.That(entities.EntityCount, Is.EqualTo(beforeInvalidLoad));

            // Local IDs can cross the wire independently of the owner. The owner
            // must use NetEntity in the eventual geometry snapshot, never raw EntityUid.
            var serializer = server.Resolve<IRobustSerializer>();
            using var stream = new MemoryStream();
            serializer.Serialize(stream, key.LayerId);
            stream.Position = 0;
            Assert.That(serializer.Deserialize<LayerId>(stream), Is.EqualTo(key.LayerId));
        });
    }
}
