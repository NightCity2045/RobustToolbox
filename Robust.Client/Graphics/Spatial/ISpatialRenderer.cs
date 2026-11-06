using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Shared.Graphics;
using Robust.Shared.Maths;

namespace Robust.Client.Graphics.Spatial;

public enum SpatialAlphaMode : byte { Opaque, Cutout, Transparent }
public enum SpatialLightKind : byte { Point, Spot }
public enum SpatialTextureSampling : byte { Nearest, PixelMip }

/// <summary>Content-provided appearance; linear color/emission, repeat scale in frame-local metres.</summary>
public sealed record SpatialRenderMaterial(Texture? Texture, Vector4 Color, Vector3 Emission,
    Vector2 Repeat, SpatialAlphaMode AlphaMode = SpatialAlphaMode.Opaque, float AlphaCutoff = 0.5f, bool DoubleSided = false,
    SpatialTextureSampling Sampling = SpatialTextureSampling.Nearest, bool PremultipliedAlpha = false)
{
    internal void Validate()
    {
        if (!Enum.IsDefined(AlphaMode) || !Enum.IsDefined(Sampling) || !Finite(Color) || Color.X < 0 || Color.Y < 0 || Color.Z < 0 || Color.W < 0 || Color.W > 1
            || !Finite(new Vector4(Emission, 0)) || Emission.X < 0 || Emission.Y < 0 || Emission.Z < 0
            || !float.IsFinite(Repeat.X) || !float.IsFinite(Repeat.Y) || Repeat.X <= 0 || Repeat.Y <= 0
            || !float.IsFinite(AlphaCutoff) || AlphaCutoff < 0 || AlphaCutoff > 1)
            throw new ArgumentException("Invalid spatial material parameters.");
    }
    private static bool Finite(Vector4 v) => float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z) && float.IsFinite(v.W);
}

/// <summary>World-space light; spot angles are cosines. Selection is independent of geometry authority.</summary>
public readonly record struct SpatialRenderLight(SpatialLightKind Kind, Vector3 Position, float Radius,
    Vector3 Color, Vector3 Direction, float InnerConeCosine = 1, float OuterConeCosine = 0, bool CastsShadows = false)
{
    internal void Validate()
    {
        if (!Enum.IsDefined(Kind) || !Finite(Position) || !Finite(Color) || Color.X < 0 || Color.Y < 0 || Color.Z < 0
            || !float.IsFinite(Radius) || Radius <= 0 || Kind == SpatialLightKind.Spot &&
            (!Finite(Direction) || MathF.Abs(Direction.LengthSquared() - 1) > 1e-5f
                || !float.IsFinite(InnerConeCosine) || !float.IsFinite(OuterConeCosine)
                || OuterConeCosine < -1 || InnerConeCosine > 1 || InnerConeCosine <= OuterConeCosine))
            throw new ArgumentException("Invalid spatial light parameters.");
    }
    private static bool Finite(Vector3 v) => float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z);
}

public readonly record struct SpatialRenderInstance(SpatialChunkMesh Mesh, RigidTransform3 FramePose);

/// <summary>One authorized immutable draw selection; it never instantiates entities or stores authoritative poses.</summary>
public sealed class SpatialRenderScene
{
    public IReadOnlyList<SpatialRenderInstance> Instances { get; }
    public IReadOnlyList<SpatialVisualInstance> Visuals { get; }
    public IReadOnlyDictionary<string, SpatialRenderMaterial> Materials { get; }
    public IReadOnlyList<SpatialRenderLight> Lights { get; }
    public Vector3 Ambient { get; }
    public Vector4 ClearColor { get; }

    public SpatialRenderScene(IEnumerable<SpatialRenderInstance> instances, IReadOnlyDictionary<string, SpatialRenderMaterial> materials,
        IEnumerable<SpatialRenderLight> lights, Vector3 ambient, Vector4 clearColor,
        IEnumerable<SpatialVisualInstance>? visuals = null)
    {
        ArgumentNullException.ThrowIfNull(instances); ArgumentNullException.ThrowIfNull(materials); ArgumentNullException.ThrowIfNull(lights);
        var copy = instances.ToArray();
        if (copy.Any(i => i.Mesh == null || !i.FramePose.IsValid)
            || copy.Select(i => (i.Mesh.Source.OwnerFrame, i.Mesh.Key)).Distinct().Count() != copy.Length)
            throw new ArgumentException("A spatial scene needs valid poses and one instance per canonical chunk.");
        var bindings = new Dictionary<string, SpatialRenderMaterial>(materials, StringComparer.Ordinal);
        foreach (var material in bindings.Values) { ArgumentNullException.ThrowIfNull(material); material.Validate(); }
        if (copy.Any(i => i.Mesh.Faces.Any(f => !bindings.ContainsKey(f.MaterialProfile))))
            throw new ArgumentException("Every visible material profile requires an explicit binding.");
        var visualArray = visuals?.ToArray() ?? Array.Empty<SpatialVisualInstance>();
        if (visualArray.Any(v => v == null || !bindings.ContainsKey(v.MaterialProfile)))
            throw new ArgumentException("Every spatial visual requires an explicit material binding.");
        var lightArray = lights.ToArray();
        foreach (var light in lightArray) light.Validate();
        if (!float.IsFinite(ambient.X) || !float.IsFinite(ambient.Y) || !float.IsFinite(ambient.Z)
            || ambient.X < 0 || ambient.Y < 0 || ambient.Z < 0 || !float.IsFinite(clearColor.X) || !float.IsFinite(clearColor.Y)
            || !float.IsFinite(clearColor.Z) || !float.IsFinite(clearColor.W) || clearColor.W < 0 || clearColor.W > 1)
            throw new ArgumentException("Invalid scene ambient/clear colors.");
        Instances = Array.AsReadOnly(copy);
        Visuals = Array.AsReadOnly(visualArray);
        Materials = new System.Collections.ObjectModel.ReadOnlyDictionary<string, SpatialRenderMaterial>(bindings);
        Lights = Array.AsReadOnly(lightArray);
        Ambient = ambient; ClearColor = clearColor;
    }
}

/// <summary>Bounded point/spot shadow quality; zero lights disables shadows without changing scene authority.</summary>
public readonly record struct SpatialShadowSettings(int MaximumLights, int Resolution, float NearPlane, float DepthBias)
{
    internal void Validate(int maximumPixels)
    {
        if (MaximumLights == 0) return;
        // GLSL 1.40 minimum uniform capacity accommodates four six-face lights.
        if (MaximumLights < 0 || MaximumLights > 4 || Resolution < 16 || Resolution > 1024
            || 6L * MaximumLights * Resolution * Resolution > maximumPixels
            || !float.IsFinite(NearPlane) || NearPlane <= 0 || !float.IsFinite(DepthBias) || DepthBias < 0 || DepthBias >= 1)
            throw new ArgumentOutOfRangeException(nameof(MaximumLights), "Invalid spatial shadow profile or atlas pixel budget.");
    }
}

public readonly record struct SpatialRendererLimits(long MaximumMeshBytes, int UploadBytesPerFrame, int MaximumPixels,
    SpatialShadowSettings Shadows = default, int MaximumVisuals = 16384)
{
    internal void Validate()
    {
        if (MaximumMeshBytes <= 0 || MaximumMeshBytes > int.MaxValue || UploadBytesPerFrame <= 0 || MaximumPixels <= 0 || MaximumVisuals <= 0)
            throw new ArgumentOutOfRangeException(nameof(MaximumMeshBytes), "Renderer budgets must be positive.");
        Shadows.Validate(MaximumPixels);
    }
}

public enum SpatialRenderStatus : byte { Ready, Uploading, BudgetExceeded }
public readonly record struct SpatialRenderResult(SpatialRenderStatus Status, int VisibleChunks, int DrawCalls,
    long ResidentMeshBytes, int UploadedBytes, int ShadowFaces = 0, int VisibleVisuals = 0);

[NotContentImplementable]
public interface ISpatialRenderer : IDisposable
{
    IRenderTexture Target { get; }
    void Resize(Vector2i size);
    /// <summary>Atomically replace shader programs. A compile failure preserves the working programs.</summary>
    void ReloadShaders();
    SpatialRenderResult Render(PerspectiveCamera camera, SpatialRenderScene scene);
}
