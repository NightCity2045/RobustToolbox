using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;

namespace Robust.Shared.Spatial.Physics;

internal readonly record struct SpatialBackendContact(SpatialBackendBodyId A, SpatialBackendPart SourceA,
    SpatialBackendBodyId B, SpatialBackendPart SourceB, Vector3 Point, Vector3 Normal,
    Vector3 RelativeVelocity, float Depth);
internal readonly record struct SpatialBackendContactEvent(bool Started, SpatialBackendContact Contact);

internal sealed partial class BepuSpatialPhysicsBackend
{
    private readonly record struct ContactKey(SpatialBackendBodyId A, int ChildA, SpatialBackendBodyId B, int ChildB);
    private Dictionary<ContactKey, SpatialBackendContact> _contacts = new();
    private Dictionary<ContactKey, SpatialBackendContact> _stepContacts = new();
    private readonly HashSet<SpatialBackendBodyId> _changedContactBodies = new();
    private readonly List<SpatialBackendContactEvent> _contactEvents = new();

    public IReadOnlyList<SpatialBackendContactEvent> DrainContactEvents()
    {
        ThrowIfDisposed();
        var result = _contactEvents.ToArray();
        _contactEvents.Clear();
        return result;
    }

    private void BeginContacts()
    {
        _stepContacts.Clear();
        // Sleeping pairs do not enter narrowphase. Retain them unless an explicit
        // pose change invalidated the cached contact; sleeping is not separation.
        foreach (var (key, contact) in _contacts)
            if (!_changedContactBodies.Contains(key.A) && !_changedContactBodies.Contains(key.B)
                && ((!IsAwake(key.A) && !IsAwake(key.B)) || FrozenPair(key)))
                _stepContacts.Add(key, contact);
        _changedContactBodies.Clear();
    }

    private bool IsAwake(SpatialBackendBodyId id)
    {
        var entry = _entries[id];
        return entry.Definition.Motion != SpatialBackendMotion.Static
            && _simulation.Bodies.GetBodyReference(entry.Body).Awake;
    }

    private bool FrozenPair(ContactKey key)
    {
        var a = _entries[key.A];
        var b = _entries[key.B];
        return (a.Paused || b.Paused)
            && (a.Paused || a.Definition.Motion == SpatialBackendMotion.Static)
            && (b.Paused || b.Definition.Motion == SpatialBackendMotion.Static);
    }

    private void EndContacts()
    {
        foreach (var (key, contact) in _contacts)
            if (!_stepContacts.ContainsKey(key))
                _contactEvents.Add(new SpatialBackendContactEvent(false, contact));
        foreach (var (key, contact) in _stepContacts)
            if (!_contacts.ContainsKey(key))
                _contactEvents.Add(new SpatialBackendContactEvent(true, contact));
        (_contacts, _stepContacts) = (_stepContacts, _contacts);
    }

    private void RemoveBodyContacts(SpatialBackendBodyId body)
    {
        foreach (var (key, contact) in _contacts.ToArray())
        {
            if (key.A != body && key.B != body)
                continue;
            _contactEvents.Add(new SpatialBackendContactEvent(false, contact));
            _contacts.Remove(key);
            _stepContacts.Remove(key);
        }
        _changedContactBodies.Remove(body);
    }

    private void RecordChildContacts(CollidablePair pair, int childA, int childB, ref ConvexContactManifold manifold)
    {
        var a = GetEntry(pair.A);
        var b = GetEntry(pair.B);
        var poseA = SolverPose(a);
        ref var compoundA = ref _simulation.Shapes.GetShape<BigCompound>(a.Shape.Index);
        // This callback precedes BEPU's NonconvexReduction, which adds the child
        // offset later. The incoming witness is relative to the child A origin.
        var childOrigin = poseA.Position + Vector3.Transform(compoundA.Children[childA].LocalPose.Position, poseA.Orientation);
        var key = a.Id.Value < b.Id.Value
            ? new ContactKey(a.Id, childA, b.Id, childB) : new ContactKey(b.Id, childB, a.Id, childA);
        // Retain an established touch across sub-millimetre solver roundoff.
        // New speculative pairs must still reach nonnegative penetration first.
        var minimumDepth = _contacts.ContainsKey(key) ? -0.0001f : 0;
        for (var i = 0; i < manifold.Count; i++)
        {
            manifold.GetContact(i, out var offset, out var normal, out var depth, out _);
            if (depth < minimumDepth)
                continue; // Speculative proximity is not a physical touch.
            var point = childOrigin + offset;
            var relative = PointVelocity(a, point) - PointVelocity(b, point);
            var contact = new SpatialBackendContact(a.Id, a.Definition.Parts[childA], b.Id, b.Definition.Parts[childB],
                point, normal, relative, depth);
            if (a.Id.Value > b.Id.Value)
            {
                contact = new SpatialBackendContact(b.Id, b.Definition.Parts[childB], a.Id, a.Definition.Parts[childA],
                    point, -normal, -relative, depth);
            }
            if (!_stepContacts.TryGetValue(key, out var previous) || depth > previous.Depth)
                _stepContacts[key] = contact;
        }
    }

    private RigidPose SolverPose(Entry entry) => entry.Definition.Motion == SpatialBackendMotion.Static
        ? _simulation.Statics.GetStaticReference(entry.Static).Pose
        : _simulation.Bodies.GetBodyReference(entry.Body).Pose;

    private Vector3 PointVelocity(Entry entry, Vector3 point)
    {
        if (entry.Definition.Motion == SpatialBackendMotion.Static)
            return Vector3.Zero;
        var body = _simulation.Bodies.GetBodyReference(entry.Body);
        return body.Velocity.Linear + Vector3.Cross(body.Velocity.Angular, point - body.Pose.Position);
    }
}
