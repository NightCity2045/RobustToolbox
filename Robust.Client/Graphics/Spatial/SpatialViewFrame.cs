using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;

namespace Robust.Client.Graphics.Spatial;

/// <summary>
/// Implemented by content: resolves current authorized view, scene and vision
/// effects for the requested physical render size. Null means no ready authorized
/// view. Does not grant subscriptions or instantiate an alternate world.
/// </summary>
public interface IWorldViewProvider
{
    SpatialViewFrame? GetFrame(Vector2i renderSize);
}

/// <summary>One immutable presentation selection of an existing world.</summary>
public abstract class SpatialViewFrame
{
    public EntityUid World { get; }
    public IReadOnlyList<ShaderInstance> Effects { get; }

    private protected SpatialViewFrame(EntityUid world, IEnumerable<ShaderInstance>? effects)
    {
        if (!world.IsValid()) throw new ArgumentException("A view requires an existing world identity.");
        World = world;
        var copy = effects?.ToArray() ?? Array.Empty<ShaderInstance>();
        if (copy.Any(p => p == null)) throw new ArgumentException("A view effect cannot be null.");
        Effects = Array.AsReadOnly(copy);
    }
}

public sealed class SpatialPerspectiveFrame : SpatialViewFrame
{
    public PerspectiveCamera Camera { get; }
    public SpatialRenderScene Scene { get; }

    public SpatialPerspectiveFrame(EntityUid world, PerspectiveCamera camera, SpatialRenderScene scene,
        IEnumerable<ShaderInstance>? effects = null) : base(world, effects)
    {
        ArgumentNullException.ThrowIfNull(scene);
        if (!camera.Pose.IsValid || camera.AspectRatio <= 0) throw new ArgumentException("Invalid perspective view.");
        Camera = camera; Scene = scene;
    }
}

public sealed class SpatialSliceFrame : SpatialViewFrame
{
    public TopDownSliceView View { get; }
    public SpatialGeometrySnapshot Geometry { get; }
    public IReadOnlyDictionary<string, Color> Palette { get; }
    public IReadOnlyList<SpatialSliceEntity> Entities { get; }
    public IReadOnlyList<SpatialSliceLine> Diagnostics { get; }
    public Color ClearColor { get; }

    public SpatialSliceFrame(EntityUid world, TopDownSliceView view, SpatialGeometrySnapshot geometry,
        IReadOnlyDictionary<string, Color> palette, IEnumerable<SpatialSliceEntity> entities,
        IEnumerable<SpatialSliceLine> diagnostics, Color clearColor,
        IEnumerable<ShaderInstance>? effects = null) : base(world, effects)
    {
        ArgumentNullException.ThrowIfNull(geometry); ArgumentNullException.ThrowIfNull(palette);
        ArgumentNullException.ThrowIfNull(entities); ArgumentNullException.ThrowIfNull(diagnostics);
        if (view.Size.X <= 0 || view.Size.Y <= 0 || view.Layer.OwnerFrame != geometry.OwnerFrame
            || !geometry.Layers.ContainsKey(view.Layer.Layer)) throw new ArgumentException("Invalid slice view selection.");
        View = view; Geometry = geometry; ClearColor = clearColor;
        Palette = new System.Collections.ObjectModel.ReadOnlyDictionary<string, Color>(new Dictionary<string, Color>(palette, StringComparer.Ordinal));
        Entities = Array.AsReadOnly(entities.ToArray()); Diagnostics = Array.AsReadOnly(diagnostics.ToArray());
    }
}
