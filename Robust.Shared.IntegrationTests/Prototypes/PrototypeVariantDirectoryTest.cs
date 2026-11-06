using NUnit.Framework;
using Robust.Shared.ContentPack;
using Robust.Shared.IoC;
using Robust.Shared.Prototypes;
using Robust.Shared.Utility;
using Robust.UnitTesting.Server;

namespace Robust.Shared.IntegrationTests.Prototypes;

[TestFixture]
public sealed class PrototypeVariantDirectoryTest
{
    [TearDown]
    public void TearDown() => IoCManager.Clear();

    [Test]
    public void ParallelFilesRetainEveryVariantCollectionAcrossLoads()
    {
        const int files = 512;
        var root = new MemoryContentRoot();
        for (var batch = 0; batch < 2; batch++)
        for (var i = 0; i < files; i++)
        {
            root.AddOrUpdateFile(new ResPath($"/Batch{batch}/{i}.yml"), $"""
                - type: entity
                  id: !type:CreateVariants
                    values: [Parallel{batch}_{i}_A, Parallel{batch}_{i}_B, Parallel{batch}_{i}_C]
                """);
        }

        var sim = RobustServerSimulation.NewSimulation()
            .AddRoot(factory => factory.AddRoot(new ResPath("/"), root))
            .RegisterPrototypes(prototypes => prototypes.LoadDirectory(new ResPath("/Batch0")))
            .InitializeInstance();
        var manager = sim.Resolve<IPrototypeManager>();
        Check(0);
        // The second load copies the previously frozen collection index before
        // concurrent files extend it. Neither old nor new groups may disappear.
        manager.LoadDirectory(new ResPath("/Batch1"));
        manager.ResolveResults();
        Check(0);
        Check(1);

        void Check(int batch)
        {
            for (var i = 0; i < files; i++)
            {
                var expected = new[] { $"Parallel{batch}_{i}_A", $"Parallel{batch}_{i}_B", $"Parallel{batch}_{i}_C" };
                foreach (var id in expected)
                {
                    Assert.That(manager.HasIndex<EntityPrototype>(id), Is.True);
                    Assert.That(manager.TryGetVariantCollection<EntityPrototype>(id, out var variants), Is.True);
                    Assert.That(variants!.Select(v => v.Id), Is.EqualTo(expected));
                }
            }
        }
    }
}
