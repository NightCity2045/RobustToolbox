using NUnit.Framework;
using Robust.Client.Graphics.Spatial;
using Robust.Shared.Maths;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;

namespace Robust.Client.Tests.Graphics;

[TestFixture]
public sealed class SpatialTextureMipChainTest
{
    private static readonly SpatialTextureBuildLimits Limits = new(64, 64 * 64 * 8);

    [Test]
    public void AtlasNeighborsCannotEnterIsolatedMipLevels()
    {
        using var source = new Image<Rgba32>(12, 4, new Rgba32(255, 0, 255, 255));
        for (var y = 0; y < 4; y++)
        for (var x = 4; x < 8; x++) source[x, y] = new Rgba32(0, 255, 0, 255);
        var chain = SpatialTextureMipChain.Build(source, new UIBox2i(4, 0, 8, 4), Limits);
        Assert.That(chain.Levels.Select(l => (l.Width, l.Height)), Is.EqualTo(new[] { (4, 4), (2, 2), (1, 1) }));
        Assert.That(chain.Bytes, Is.EqualTo(84));
        source[4, 0] = new Rgba32(255, 0, 0, 255);
        foreach (var level in chain.Levels)
        foreach (var pixel in level.Pixels.Span) Assert.That(pixel, Is.EqualTo(new Rgba32(0, 255, 0, 255)));
    }

    [Test]
    public void ReductionUsesLinearLightAlphaWeightAndEveryOddEdgeTexel()
    {
        using var source = new Image<Rgba32>(2, 1);
        source[0, 0] = new Rgba32(0, 0, 0, 255); source[1, 0] = new Rgba32(255, 255, 255, 255);
        var gamma = SpatialTextureMipChain.Build(source, new UIBox2i(0, 0, 2, 1), Limits);
        Assert.That(gamma.Levels[1].Pixels.Span[0].R, Is.EqualTo(188));
        source[0, 0] = new Rgba32(255, 0, 0, 255); source[1, 0] = new Rgba32(0, 0, 255, 0);
        var alpha = SpatialTextureMipChain.Build(source, new UIBox2i(0, 0, 2, 1), Limits);
        Assert.That(alpha.Levels[1].Pixels.Span[0], Is.EqualTo(new Rgba32(255, 0, 0, 128)));
        using var odd = new Image<Rgba32>(3, 1);
        odd[0, 0] = new Rgba32(255, 0, 0, 255); odd[1, 0] = new Rgba32(0, 255, 0, 255); odd[2, 0] = new Rgba32(0, 0, 255, 255);
        var edges = SpatialTextureMipChain.Build(odd, new UIBox2i(0, 0, 3, 1), Limits, srgb: false);
        Assert.That(edges.Levels[1].Pixels.Span[0], Is.EqualTo(new Rgba32(85, 85, 85, 255)));
    }

    [Test]
    public void CoverageCorrectionAndBudgetsAreExplicit()
    {
        using var source = new Image<Rgba32>(2, 2, new Rgba32(255, 255, 255, 255));
        source[0, 0] = new Rgba32(255, 0, 255, 0);
        var chain = SpatialTextureMipChain.Build(source, new UIBox2i(0, 0, 2, 2), Limits, alphaCutoff: 0.9f);
        Assert.That(chain.Levels[1].Pixels.Span[0].A / 255f, Is.GreaterThanOrEqualTo(0.9f));
        Assert.That(chain.Levels[1].Pixels.Span[0].R, Is.EqualTo(255));
        Assert.That(chain.Levels[1].Pixels.Span[0].G, Is.EqualTo(255), "Invisible magenta may not tint the white silhouette.");
        Assert.Throws<InvalidOperationException>(() => SpatialTextureMipChain.Build(source, new UIBox2i(0, 0, 2, 2), new SpatialTextureBuildLimits(64, 16)));
        Assert.Throws<ArgumentException>(() => SpatialTextureMipChain.Build(source, new UIBox2i(0, 0, 3, 2), Limits));
        Assert.Throws<OperationCanceledException>(() => SpatialTextureMipChain.Build(source, new UIBox2i(0, 0, 2, 2), Limits,
            cancellationToken: new CancellationToken(true)));
    }
}
