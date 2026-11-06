using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;

namespace Robust.Client.Graphics.Spatial;

public readonly record struct SpatialParticleLimits(int MaximumParticles, float MaximumLifetime);

/// <summary>
/// A decorative emission in world coordinates or in an explicitly attached
/// frame. Content supplies resources, colors, motion and authorized emission
/// events. These particles never simulate projectiles, damage or collision bodies.
/// </summary>
public sealed record SpatialParticleEmission(SpatialSpriteProfile Profile, RigidTransform3 Pose,
    string MaterialProfile, float Lifetime, Vector3 Velocity, Vector3 Acceleration,
    Vector2 EndSize, Vector4 StartColor, Vector4 EndColor,
    EntityUid? Frame = null, EntityUid? Owner = null, Vector3 AngularVelocity = default);

/// <summary>
/// Per-world bounded decoration pool. Motion is analytical in XYZ, independent
/// of rendering frequency and observer orientation. Caller owns authorization and
/// invokes RemoveOwner on revocation/deletion; missing attached frames expire.
/// All methods run on its owning client thread. No ECS or physical world is owned.
/// </summary>
public sealed class SpatialParticlePool : IDisposable
{
    private readonly SpatialParticleLimits _limits;
    private readonly int _thread = Environment.CurrentManagedThreadId;
    private readonly Dictionary<ulong, Particle> _particles = new();
    private ulong _next;
    private bool _disposed;
    public int Count => _particles.Count;
    public long RejectedByBudget { get; private set; }
    public long InvalidMotion { get; private set; }

    public SpatialParticlePool(SpatialParticleLimits limits)
    {
        if (limits.MaximumParticles < 0 || !float.IsFinite(limits.MaximumLifetime) || limits.MaximumLifetime <= 0)
            throw new ArgumentOutOfRangeException(nameof(limits));
        _limits = limits;
    }

    public bool TryEmit(SpatialParticleEmission emission, out ulong handle)
    {
        CheckThread(); ArgumentNullException.ThrowIfNull(emission); handle = 0;
        emission.Profile.Validate();
        if (!emission.Pose.IsValid || string.IsNullOrWhiteSpace(emission.MaterialProfile)
            || !float.IsFinite(emission.Lifetime) || emission.Lifetime <= 0 || emission.Lifetime > _limits.MaximumLifetime
            || !SpatialVisualInstance.Finite(emission.Velocity) || !SpatialVisualInstance.Finite(emission.Acceleration)
            || !float.IsFinite(emission.EndSize.X) || !float.IsFinite(emission.EndSize.Y) || emission.EndSize.X <= 0 || emission.EndSize.Y <= 0
            || !SpatialVisualInstance.ValidColor(emission.StartColor) || !SpatialVisualInstance.ValidColor(emission.EndColor)
            || !SpatialVisualInstance.Finite(emission.AngularVelocity) || !float.IsFinite(emission.AngularVelocity.Length())
            || emission.Frame is { } frame && !frame.IsValid() || emission.Owner is { } owner && !owner.IsValid())
            throw new ArgumentException("Invalid spatial particle emission.");
        if (Count >= _limits.MaximumParticles) { RejectedByBudget++; return false; }
        handle = checked(++_next);
        _particles.Add(handle, new Particle(emission));
        return true;
    }

    public void Advance(double seconds)
    {
        CheckThread();
        if (!double.IsFinite(seconds) || seconds < 0) throw new ArgumentOutOfRangeException(nameof(seconds));
        foreach (var (id, particle) in _particles.ToArray())
        {
            particle.Age += seconds;
            if (particle.Age >= particle.Emission.Lifetime) _particles.Remove(id);
        }
    }

    /// <summary>Produces per-observer draw records using this frame's current authorized frame poses.</summary>
    public IReadOnlyList<SpatialVisualInstance> CreateVisuals(PerspectiveCamera camera, IReadOnlyDictionary<EntityUid, RigidTransform3> frames)
    {
        CheckThread(); ArgumentNullException.ThrowIfNull(frames);
        var result = new List<SpatialVisualInstance>(Count);
        foreach (var (id, particle) in _particles.ToArray())
        {
            var e = particle.Emission;
            var parent = RigidTransform3.Identity;
            if (e.Frame is { } frame)
            {
                if (!frames.TryGetValue(frame, out parent)) { _particles.Remove(id); continue; }
                if (!parent.IsValid) throw new ArgumentException("Invalid particle frame pose.");
            }
            var age = particle.Age;
            var halfAgeSquared = age * age * 0.5;
            var position = new Vector3(
                (float) (e.Pose.Position.X + e.Velocity.X * age + e.Acceleration.X * halfAgeSquared),
                (float) (e.Pose.Position.Y + e.Velocity.Y * age + e.Acceleration.Y * halfAgeSquared),
                (float) (e.Pose.Position.Z + e.Velocity.Z * age + e.Acceleration.Z * halfAgeSquared));
            position = Vector3.Transform(position, parent.Rotation) + parent.Position;
            var angularSpeed = e.AngularVelocity.Length();
            var spin = angularSpeed == 0 ? Quaternion.Identity : Quaternion.CreateFromAxisAngle(e.AngularVelocity / angularSpeed,
                (float) (angularSpeed * age % (2 * Math.PI)));
            var rotation = Quaternion.Concatenate(Quaternion.Concatenate(e.Pose.Rotation, spin), parent.Rotation);
            if (!SpatialVisualInstance.Finite(position)) { _particles.Remove(id); InvalidMotion++; continue; }
            var fraction = (float) (age / e.Lifetime);
            var profile = e.Profile with { Size = Vector2.Lerp(e.Profile.Size, e.EndSize, fraction) };
            result.Add(profile.Resolve(new RigidTransform3(position, rotation), camera, e.MaterialProfile, e.Owner,
                modulate: Vector4.Lerp(e.StartColor, e.EndColor, fraction)));
        }
        return result.AsReadOnly();
    }

    public bool Remove(ulong handle) { CheckThread(); return _particles.Remove(handle); }
    public void RemoveOwner(EntityUid owner)
    {
        CheckThread();
        foreach (var id in _particles.Where(p => p.Value.Emission.Owner == owner).Select(p => p.Key).ToArray()) _particles.Remove(id);
    }
    public void Clear() { CheckThread(); _particles.Clear(); }
    private void CheckThread()
    {
        if (_disposed) throw new ObjectDisposedException(nameof(SpatialParticlePool));
        if (_thread != Environment.CurrentManagedThreadId) throw new InvalidOperationException("Particle pool requires its owning client thread.");
    }
    public void Dispose() { if (_disposed) return; Clear(); _disposed = true; }
    private sealed class Particle(SpatialParticleEmission emission)
    {
        public readonly SpatialParticleEmission Emission = emission;
        public double Age;
    }
}
