using System.Collections.Concurrent;
using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Moq;
using NUnit.Framework;
using Robust.Shared.ContentPack;
using Robust.Shared.GameObjects;
using Robust.Shared.Log;

namespace Robust.Shared.Tests.Spatial;

/// <summary>
/// E00: shipping a trusted backend must not grant content direct access to it.
/// Compiles real content-shaped assemblies and runs the normal IL/type verifier.
/// The physics execution cases are in <see cref="BepuQualificationTest"/>.
/// </summary>
[TestFixture]
public sealed class BepuSandboxQualificationTest
{
    [TestCase("new Robust.Shared.GameObjects.EntityUid(1).IsValid()", true)]
    [TestCase("new BepuPhysics.Collidables.Sphere(1).Radius > 0", false)]
    public void SandboxPreservesEngineBoundary(string expression, bool allowed)
    {
        var platformAssemblies = (string) AppContext.GetData("TRUSTED_PLATFORM_ASSEMBLIES")!;
        var paths = platformAssemblies.Split(Path.PathSeparator).Concat(new[]
        {
            typeof(EntityUid).Assembly.Location,
            typeof(Sphere).Assembly.Location,
            typeof(BufferPool).Assembly.Location
        }).Distinct();
        var compilation = CSharpCompilation.Create("Content.SpatialQualification",
            new[] { CSharpSyntaxTree.ParseText($$"""
                namespace Content.SpatialQualification;
                public static class Probe
                {
                    public static bool Run() => {{expression}};
                }
                """) },
            paths.Select(path => MetadataReference.CreateFromFile(path)),
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));
        using var assembly = new MemoryStream();
        var result = compilation.Emit(assembly);
        Assert.That(result.Success, Is.True, string.Join(Environment.NewLine, result.Diagnostics));
        assembly.Position = 0;

        var errors = new ConcurrentQueue<string>();
        var sawmill = new Mock<ISawmill>();
        sawmill.Setup(log => log.Error(It.IsAny<string>())).Callback<string>(errors.Enqueue);
        var checker = new AssemblyTypeChecker(Mock.Of<IResourceManager>(), sawmill.Object);
        Assert.That(checker.CheckAssembly(assembly), Is.EqualTo(allowed), string.Join(Environment.NewLine, errors));
        if (!allowed)
        {
            // Reject the backend reference itself, not a broken assembly or an unrelated IL error.
            Assert.That(errors.Any(message => message.Contains("BepuPhysics", StringComparison.Ordinal)), Is.True,
                string.Join(Environment.NewLine, errors));
        }
    }
}
