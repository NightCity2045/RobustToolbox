using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;
using Robust.Shared.Spatial;

namespace Robust.Client.Graphics.Spatial;

/// <summary>An authorized entity's existing layered sprite, with an explicit authoring-plane assignment.</summary>
public readonly record struct SpatialSliceEntity(EntityUid Entity, SpatialLayerReference Layer,
    RigidTransform3 WorldPose, Vector2 SpriteScale);
public readonly record struct SpatialSliceLine(SpatialLayerReference Layer, Vector3 WorldStart, Vector3 WorldEnd, Color Color);
public readonly record struct SpatialSliceSurface(SpatialSurfaceReference Surface, EntityUid? Entity,
    Box2 Footprint, float Elevation, string MaterialProfile, ulong Revision);

/// <summary>
/// A separate flat projection onto one canonical frame/layer. Existing entity
/// sprites are drawn directly through Clyde's layered 2D path, never by looking
/// down at perspective billboards. Coordinates are physical viewport pixels;
/// callers apply the same UI-to-viewport conversion for drawing and selection.
/// This value carries no observer permissions; inputs must already be authorized.
/// </summary>
public readonly struct TopDownSliceView
{
    public SpatialLayerReference Layer { get; }
    public RigidTransform3 FramePose { get; }
    public Vector2 Center { get; }
    public Vector2i Size { get; }
    public float PixelsPerMetre { get; }
    public float CutElevation { get; }

    /// <param name="cutElevation">Frame-local continuous height of the diagnostic section.</param>
    public TopDownSliceView(SpatialLayerReference layer, RigidTransform3 framePose, Vector2 center,
        Vector2i size, float pixelsPerMetre, float cutElevation)
    {
        if (!layer.OwnerFrame.IsValid() || !framePose.IsValid || !float.IsFinite(center.X) || !float.IsFinite(center.Y)
            || size.X <= 0 || size.Y <= 0 || !float.IsFinite(pixelsPerMetre) || pixelsPerMetre <= 0 || !float.IsFinite(cutElevation))
            throw new ArgumentException("Invalid authoring-plane view.");
        Layer = layer; FramePose = framePose; Center = center; Size = size;
        PixelsPerMetre = pixelsPerMetre; CutElevation = cutElevation;
    }

    public bool Includes(SpatialLayerReference layer) => layer == Layer;
    public Vector2 ProjectLocal(Vector2 position) => new(Size.X * 0.5f + (position.X - Center.X) * PixelsPerMetre,
        Size.Y * 0.5f - (position.Y - Center.Y) * PixelsPerMetre);
    public Vector2 UnprojectLocal(Vector2 pixel) => new(Center.X + (pixel.X - Size.X * 0.5f) / PixelsPerMetre,
        Center.Y - (pixel.Y - Size.Y * 0.5f) / PixelsPerMetre);
    public Vector2 ProjectWorld(Vector3 position)
    {
        var local = FramePose.InverseTransformPoint(position);
        return ProjectLocal(new Vector2(local.X, local.Y));
    }
    public Vector3 UnprojectWorld(Vector2 pixel) => FramePose.TransformPoint(new Vector3(UnprojectLocal(pixel), CutElevation));

    /// <summary>Floors below the section and boundaries intersecting it, restricted to the selected authoring layer.</summary>
    public IReadOnlyList<SpatialSliceSurface> GetSurfaces(SpatialGeometrySnapshot snapshot)
    {
        if (snapshot.OwnerFrame != Layer.OwnerFrame || !snapshot.Layers.ContainsKey(Layer.Layer))
            throw new ArgumentException("The slice requires the selected frame/layer snapshot.");
        var selectedLayer = Layer.Layer;
        var slabs = snapshot.Slabs.Values.Where(s => s.Layer == selectedLayer).Select(s => s.Surface.Surface).ToHashSet();
        var boundaries = snapshot.Boundaries.Values.Where(b => b.Key.LayerId == selectedLayer).Select(b => b.Surface).ToHashSet();
        var result = new List<SpatialSliceSurface>();
        foreach (var part in snapshot.SolidParts)
        {
            var slab = slabs.Contains(part.Surface.Surface);
            if (slab ? part.Bounds.Max.Z > CutElevation
                : !boundaries.Contains(part.Surface.Surface) || part.Bounds.Min.Z > CutElevation || part.Bounds.Max.Z <= CutElevation)
                continue;
            result.Add(new SpatialSliceSurface(part.Surface, part.Entity,
                new Box2(part.Bounds.Min.X, part.Bounds.Min.Y, part.Bounds.Max.X, part.Bounds.Max.Y),
                slab ? part.Bounds.Max.Z : CutElevation, part.MaterialProfile, part.Revision));
        }
        return Array.AsReadOnly(result.OrderBy(s => s.Elevation).ThenBy(s => s.Surface.Surface.Value).ToArray());
    }

    public SpatialSliceSurface? PickSurface(SpatialGeometrySnapshot snapshot, Vector2 pixel)
    {
        if (pixel.X < 0 || pixel.Y < 0 || pixel.X >= Size.X || pixel.Y >= Size.Y) return null;
        var local = UnprojectLocal(pixel);
        var surfaces = GetSurfaces(snapshot);
        for (var i = surfaces.Count - 1; i >= 0; i--)
            if (surfaces[i].Footprint.Contains(local)) return surfaces[i];
        return null;
    }

    /// <summary>
    /// Draw into the caller's existing Clyde target/viewport. The flat surface
    /// palette is explicit; game art remains the existing entity Sprite layers.
    /// The caller owns viewport clipping and clear color through IRenderHandle.
    /// </summary>
    public void Draw(IRenderHandle handle, SpatialGeometrySnapshot snapshot, IReadOnlyDictionary<string, Color> palette,
        IEnumerable<SpatialSliceEntity> entities, IEnumerable<SpatialSliceLine> diagnostics)
    {
        var surfaces = GetSurfaces(snapshot);
        foreach (var surface in surfaces)
            if (!palette.ContainsKey(surface.MaterialProfile)) throw new ArgumentException("Every slice surface requires a palette binding.");
        var screen = handle.DrawingHandleScreen;
        var oldTransform = screen.GetTransform(); var oldShader = screen.GetShader();
        try
        {
            screen.SetTransform(Matrix3x2.Identity); screen.UseShader(null);
            foreach (var surface in surfaces)
            {
                var topLeft = ProjectLocal(new Vector2(surface.Footprint.Left, surface.Footprint.Top));
                var bottomRight = ProjectLocal(new Vector2(surface.Footprint.Right, surface.Footprint.Bottom));
                screen.DrawRect(new UIBox2(topLeft, bottomRight), palette[surface.MaterialProfile]);
            }
            foreach (var entity in entities)
            {
                if (!Includes(entity.Layer)) continue;
                if (!entity.WorldPose.IsValid || !float.IsFinite(entity.SpriteScale.X) || !float.IsFinite(entity.SpriteScale.Y))
                    throw new ArgumentException("Invalid slice entity pose or sprite scale.");
                var localRight = FramePose.InverseTransformVector(entity.WorldPose.TransformVector(Vector3.UnitX));
                var rotation = new Angle(Math.Atan2(localRight.Y, localRight.X));
                handle.DrawEntity(entity.Entity, ProjectWorld(entity.WorldPose.Position),
                    entity.SpriteScale * (PixelsPerMetre / EyeManager.PixelsPerMeter), rotation);
            }
            foreach (var line in diagnostics)
                if (Includes(line.Layer)) screen.DrawLine(ProjectWorld(line.WorldStart), ProjectWorld(line.WorldEnd), line.Color);
        }
        finally { screen.SetTransform(oldTransform); screen.UseShader(oldShader); }
    }
}
