using System.Numerics;
using NUnit.Framework;
using Robust.Client.Graphics;
using Robust.Client.Graphics.Clyde;
using Robust.Client.Graphics.Spatial;
using Robust.Shared.Maths;
using SixLabors.ImageSharp.PixelFormats;

namespace Robust.Client.Tests.Graphics;

[TestFixture]
public sealed class SpatialSpriteCacheTest
{
    [Test]
    public void CompleteSignaturesShareReservationsAndPinnedEntriesCannotBeEvicted()
    {
        var clyde = new ClydeHeadless();
        using var texture = clyde.CreateBlankTexture<Rgba32>(new Vector2i(32, 32));
        using var cache = new SpatialSpriteCache(clyde, new SpatialSpriteCacheLimits(2, 8192, 32, 4));
        SpatialSpriteComposition Appearance(Color tint) => new(new Vector2i(32, 32), new[] {
            new SpatialSpriteLayer(texture, new UIBox2(0, 0, 32, 32), Matrix3x2.Identity, tint) });
        using var first = cache.Request(Appearance(Color.White))!;
        using var shared = cache.Request(Appearance(Color.White))!;
        using var changed = cache.Request(Appearance(Color.Red))!;
        Assert.That(cache.Count, Is.EqualTo(2)); Assert.That(cache.ReservedBytes, Is.EqualTo(8192));
        Assert.That(first.IsReady, Is.False);
        Assert.Throws<InvalidOperationException>(() => _ = first.Texture);
        Assert.That(cache.Request(Appearance(Color.Blue)), Is.Null, "Active appearances cannot silently disappear to satisfy a budget.");
        first.Dispose();
        Assert.That(cache.Request(Appearance(Color.Blue)), Is.Null, "The second observer still pins the shared appearance.");
        shared.Dispose();
        using var replacement = cache.Request(Appearance(Color.Blue));
        Assert.That(replacement, Is.Not.Null); Assert.That(changed.IsValid, Is.True);
        Assert.That(cache.Count, Is.EqualTo(2)); Assert.That(cache.ReservedBytes, Is.EqualTo(8192));
        cache.Clear();
        Assert.That(changed.IsValid, Is.False); Assert.That(replacement!.IsValid, Is.False);
        Assert.That(cache.ReservedBytes, Is.Zero);
    }

    [Test]
    public void ResourceReloadInvalidatesAtlasLeasesAndNewFramesUseSeparateSignatures()
    {
        var clyde = new ClydeHeadless();
        using var texture = clyde.CreateBlankTexture<Rgba32>(new Vector2i(64, 64));
        var atlas = new AtlasTexture(texture, new UIBox2(0, 0, 32, 32));
        using var cache = new SpatialSpriteCache(clyde, new SpatialSpriteCacheLimits(4, 16384, 64, 3));
        var firstLayer = new SpatialSpriteLayer(atlas, new UIBox2(0, 0, 32, 32), Matrix3x2.Identity, Color.White);
        var layers = new[] { firstLayer };
        var appearance = new SpatialSpriteComposition(new Vector2i(32, 32), layers);
        layers[0] = firstLayer with { Tint = Color.Red };
        Assert.That(appearance.Layers[0].Tint, Is.EqualTo(Color.White), "Signature owns its input sequence.");
        using var first = cache.Request(appearance)!;
        using var second = cache.Request(new SpatialSpriteComposition(new Vector2i(32, 32), new[] {
            firstLayer with { Transform = Matrix3x2.CreateTranslation(1, 0) } }))!;
        Assert.That(cache.Count, Is.EqualTo(2));
        cache.Invalidate(texture);
        Assert.That(first.IsValid, Is.False); Assert.That(second.IsValid, Is.False); Assert.That(cache.Count, Is.Zero);
        using var reloaded = cache.Request(appearance)!;
        Assert.That(reloaded.IsValid, Is.True); Assert.That(reloaded.IsReady, Is.False);
        Assert.That(cache.Request(new SpatialSpriteComposition(new Vector2i(65, 1), new[] { firstLayer })), Is.Null);
        Assert.That(cache.Request(new SpatialSpriteComposition(new Vector2i(32, 32), Enumerable.Repeat(firstLayer, 4))), Is.Null);
        cache.Dispose();
        Assert.That(reloaded.IsValid, Is.False);
        Assert.Throws<ObjectDisposedException>(() => cache.Request(appearance));
    }
}
