using System;
using System.Collections.Generic;
using System.Threading;
using Robust.Shared.Maths;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;

namespace Robust.Client.Graphics.Spatial;

public readonly record struct SpatialTextureBuildLimits(int MaximumDimension, long MaximumBytes);

public sealed class SpatialTextureMipLevel
{
    public int Width { get; }
    public int Height { get; }
    public ReadOnlyMemory<Rgba32> Pixels { get; }
    internal SpatialTextureMipLevel(int width, int height, Rgba32[] pixels)
    { Width = width; Height = height; Pixels = pixels; }
}

/// <summary>
/// Detached material image pyramid. Extract an atlas region BEFORE reduction, so
/// neighboring atlas tiles cannot enter any level. Each result is uploaded as an
/// independent texture; existing shared RSI atlases are neither changed nor given
/// unsafe whole-atlas mipmaps. Colors are reduced in linear light with alpha
/// weighting, preserving straight-alpha storage and avoiding transparent fringes.
/// </summary>
public sealed class SpatialTextureMipChain
{
    public IReadOnlyList<SpatialTextureMipLevel> Levels { get; }
    public bool Srgb { get; }
    public float? AlphaCutoff { get; }
    public long Bytes { get; }

    private SpatialTextureMipChain(List<SpatialTextureMipLevel> levels, bool srgb, float? alphaCutoff, long bytes)
    { Levels = levels.AsReadOnly(); Srgb = srgb; AlphaCutoff = alphaCutoff; Bytes = bytes; }

    /// <summary>
    /// Source must not be mutated during the copy. Limits bound the resulting
    /// pyramid; construction additionally holds at most two unadjusted mip levels
    /// when alpha-coverage correction is enabled. No game state or GPU is accessed.
    /// </summary>
    public static SpatialTextureMipChain Build(Image<Rgba32> source, UIBox2i region,
        SpatialTextureBuildLimits limits, bool srgb = true, float? alphaCutoff = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(source);
        if (limits.MaximumDimension <= 0 || limits.MaximumBytes <= 0 || limits.MaximumBytes > int.MaxValue
            || region.Left < 0 || region.Top < 0 || region.Width <= 0 || region.Height <= 0
            || region.Right > source.Width || region.Bottom > source.Height
            || region.Width > limits.MaximumDimension || region.Height > limits.MaximumDimension
            || alphaCutoff is { } cutoff && (!float.IsFinite(cutoff) || cutoff <= 0 || cutoff > 1))
            throw new ArgumentException("Invalid spatial texture region, coverage threshold or profile.");
        long bytes = 0;
        for (int w = region.Width, h = region.Height; ; w = Math.Max(1, w / 2), h = Math.Max(1, h / 2))
        {
            bytes = checked(bytes + (long) w * h * 4);
            if (bytes > limits.MaximumBytes) throw new InvalidOperationException("Spatial texture pyramid exceeds its byte budget.");
            if (w == 1 && h == 1) break;
        }
        cancellationToken.ThrowIfCancellationRequested();
        var width = region.Width; var height = region.Height;
        var raw = new Rgba32[checked(width * height)];
        for (var y = 0; y < height; y++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            for (var x = 0; x < width; x++) raw[y * width + x] = source[region.Left + x, region.Top + y];
        }
        var levels = new List<SpatialTextureMipLevel> { new(width, height, raw) };
        var coverage = alphaCutoff is { } threshold ? Coverage(raw, threshold, 1, cancellationToken) : 0;
        while (width != 1 || height != 1)
        {
            var nextWidth = Math.Max(1, width / 2); var nextHeight = Math.Max(1, height / 2);
            var next = new Rgba32[nextWidth * nextHeight];
            for (var y = 0; y < nextHeight; y++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var top = (double) y * height / nextHeight; var bottom = (double) (y + 1) * height / nextHeight;
                for (var x = 0; x < nextWidth; x++)
                {
                    var left = (double) x * width / nextWidth; var right = (double) (x + 1) * width / nextWidth;
                    double red = 0, green = 0, blue = 0, alpha = 0;
                    for (var sy = (int) top; sy < Math.Ceiling(bottom); sy++)
                    for (var sx = (int) left; sx < Math.Ceiling(right); sx++)
                    {
                        var weight = (Math.Min(right, sx + 1) - Math.Max(left, sx)) * (Math.Min(bottom, sy + 1) - Math.Max(top, sy));
                        var sample = raw[sy * width + sx];
                        var weightedAlpha = sample.A / 255.0 * weight;
                        alpha += weightedAlpha;
                        red += Decode(sample.R, srgb) * weightedAlpha;
                        green += Decode(sample.G, srgb) * weightedAlpha;
                        blue += Decode(sample.B, srgb) * weightedAlpha;
                    }
                    next[y * nextWidth + x] = new Rgba32(
                        alpha == 0 ? (byte) 0 : Encode(red / alpha, srgb),
                        alpha == 0 ? (byte) 0 : Encode(green / alpha, srgb),
                        alpha == 0 ? (byte) 0 : Encode(blue / alpha, srgb),
                        Quantize(alpha / ((right - left) * (bottom - top))));
                }
            }
            var stored = next;
            if (alphaCutoff is { } cut)
            {
                stored = (Rgba32[]) next.Clone();
                PreserveCoverage(stored, cut, coverage, cancellationToken);
            }
            levels.Add(new SpatialTextureMipLevel(nextWidth, nextHeight, stored));
            // Correction is presentation-only: never feed scaled alpha into the
            // next reduction, which would accumulate a growing coverage bias.
            raw = next; width = nextWidth; height = nextHeight;
        }
        return new SpatialTextureMipChain(levels, srgb, alphaCutoff, bytes);
    }

    private static double Decode(byte channel, bool srgb)
    {
        var value = channel / 255.0;
        return !srgb ? value : value <= 0.04045 ? value / 12.92 : Math.Pow((value + 0.055) / 1.055, 2.4);
    }
    private static byte Encode(double value, bool srgb)
        => Quantize(!srgb ? value : value <= 0.0031308 ? value * 12.92 : 1.055 * Math.Pow(value, 1 / 2.4) - 0.055);
    private static byte Quantize(double value) => (byte) Math.Clamp((int) Math.Round(value * 255), 0, 255);

    private static double Coverage(Rgba32[] pixels, float threshold, double scale, CancellationToken token)
    {
        var count = 0;
        for (var i = 0; i < pixels.Length; i++)
        {
            if (i % 4096 == 0) token.ThrowIfCancellationRequested();
            if (Quantize(pixels[i].A / 255.0 * scale) / 255.0 >= threshold) count++;
        }
        return (double) count / pixels.Length;
    }

    private static void PreserveCoverage(Rgba32[] pixels, float threshold, double desired, CancellationToken token)
    {
        var best = 1.0; var error = Math.Abs(Coverage(pixels, threshold, best, token) - desired);
        double low = 0, high = 256;
        for (var i = 0; i < 24; i++)
        {
            var scale = (low + high) * 0.5;
            var coverage = Coverage(pixels, threshold, scale, token);
            var nextError = Math.Abs(coverage - desired);
            if (nextError < error || nextError == error && Math.Abs(scale - 1) < Math.Abs(best - 1))
            { error = nextError; best = scale; }
            if (coverage < desired) low = scale; else high = scale;
        }
        for (var i = 0; i < pixels.Length; i++) pixels[i].A = Quantize(pixels[i].A / 255.0 * best);
    }
}
