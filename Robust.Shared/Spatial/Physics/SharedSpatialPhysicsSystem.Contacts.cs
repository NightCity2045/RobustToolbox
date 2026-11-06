using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Robust.Shared.GameObjects;

namespace Robust.Shared.Spatial.Physics;

/// <summary>
/// World-space pre-solve contact witness. Normal points from Other toward Self;
/// RelativeVelocity is Self minus Other at the witness before the contact solve.
/// End notifications retain the last touching witness, not a new impact.
/// </summary>
public readonly record struct SpatialContact(SpatialQueryCandidate Self, SpatialQueryCandidate Other,
    Vector3 Point, Vector3 Normal, Vector3 RelativeVelocity, float Depth);

[ByRefEvent]
public readonly record struct SpatialContactStarted(SpatialContact Contact);
[ByRefEvent]
public readonly record struct SpatialContactEnded(SpatialContact Contact);

public sealed partial class SharedSpatialPhysicsSystem
{
    private readonly record struct ContactSource(EntityUid? Entity, SpatialSurfaceReference? Surface);
    private readonly record struct SemanticContactKey(EntityUid World, ContactSource Self, ContactSource Other);
    private readonly record struct SemanticContactState(int Count, SpatialContact Contact);
    private readonly Dictionary<SemanticContactKey, SemanticContactState> _semanticContacts = new();

    private void PublishContacts()
    {
        // Snapshot before notifying content: a handler may delete a body or world.
        var pending = new List<(EntityUid World, SpatialBackendContactEvent Event)>();
        foreach (var (world, backend) in _worlds)
            foreach (var ev in backend.DrainContactEvents())
                pending.Add((world, ev));
        var previous = new Dictionary<SemanticContactKey, SemanticContactState>();
        var firstStarts = new Dictionary<SemanticContactKey, SpatialContact>();
        foreach (var (world, ev) in pending)
        {
            var c = ev.Contact;
            var a = new SpatialQueryCandidate(c.SourceA.Entity, c.SourceA.Surface, c.SourceA.MaterialProfile, c.SourceA.Revision);
            var b = new SpatialQueryCandidate(c.SourceB.Entity, c.SourceB.Surface, c.SourceB.MaterialProfile, c.SourceB.Revision);
            Accumulate(world, ev.Started, new SpatialContact(a, b, c.Point, c.Normal, c.RelativeVelocity, c.Depth));
            Accumulate(world, ev.Started, new SpatialContact(b, a, c.Point, -c.Normal, -c.RelativeVelocity, c.Depth));
        }
        // Parts of one semantic surface can meet at a seam. Content receives one
        // transition per source pair, not a separate impact for every convex part.
        var notifications = new List<(bool Started, SpatialContact Contact)>();
        foreach (var (key, before) in previous)
        {
            var after = _semanticContacts[key];
            if (before.Count == 0 && firstStarts.TryGetValue(key, out var first))
                notifications.Add((true, first));
            if (after.Count == 0)
            {
                if (before.Count > 0 || firstStarts.ContainsKey(key))
                    notifications.Add((false, after.Contact));
                _semanticContacts.Remove(key);
            }
        }
        foreach (var (started, contact) in notifications)
            Send(started, contact);

        void Accumulate(EntityUid world, bool started, SpatialContact contact)
        {
            var key = new SemanticContactKey(world, new ContactSource(contact.Self.Entity, contact.Self.Surface),
                new ContactSource(contact.Other.Entity, contact.Other.Surface));
            _semanticContacts.TryGetValue(key, out var current);
            previous.TryAdd(key, current);
            if (started)
                firstStarts.TryAdd(key, contact);
            var count = current.Count + (started ? 1 : -1);
            if (count < 0)
                throw new System.InvalidOperationException("Spatial contact end has no corresponding active part.");
            _semanticContacts[key] = new SemanticContactState(count, contact);
        }
    }

    private void ForgetWorldContacts(EntityUid world)
    {
        foreach (var key in _semanticContacts.Keys.Where(k => k.World == world).ToArray())
            _semanticContacts.Remove(key);
    }

    private void Send(bool started, SpatialContact contact)
    {
        if (contact.Self.Entity is not { } entity || TerminatingOrDeleted(entity))
            return;
        if (started)
        {
            var ev = new SpatialContactStarted(contact);
            RaiseLocalEvent(entity, ref ev);
        }
        else
        {
            var ev = new SpatialContactEnded(contact);
            RaiseLocalEvent(entity, ref ev);
        }
    }
}
