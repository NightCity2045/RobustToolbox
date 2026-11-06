using System;
using System.Numerics;
using Robust.Shared.Audio.Effects;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.Audio.Sources;

/// <summary>
///     Hey look, it's AudioSource's evil twin brother!
/// </summary>
[Virtual]
[DataDefinition]
internal partial class DummyAudioSource : IAudioSource
{
    public static DummyAudioSource Instance { get; } = new();

    public void Dispose()
    {
    }

    /// <inheritdoc />
    public void Pause()
    {
    }

    /// <inheritdoc />
    public void StartPlaying()
    {
    }

    /// <inheritdoc />
    public void StopPlaying()
    {
    }

    /// <inheritdoc />
    public void Restart()
    {
    }

    /// <inheritdoc />
    public bool Playing { get; set; }

    /// <inheritdoc />
    [DataField]
    public bool Looping { get; set; }

    /// <inheritdoc />
    [DataField]
    public bool Global { get; set; }

    /// <inheritdoc />
    public Vector2 Position
    {
        get => new(Position3.X, Position3.Y);
        set { if (float.IsFinite(value.X) && float.IsFinite(value.Y)) Position3 = new Vector3(value, Position3.Z); }
    }
    private Vector3 _position3;
    public Vector3 Position3 { get => _position3; set { RequireFinite(value); _position3 = value; } }

    /// <inheritdoc />
    [DataField]
    public float Pitch { get; set; }

    /// <inheritdoc />
    public float Volume { get; set; }

    /// <inheritdoc />
    public float Gain { get; set; }

    /// <inheritdoc />
    [DataField]
    public float MaxDistance { get; set; }

    /// <inheritdoc />
    [DataField]
    public float RolloffFactor { get; set; }

    /// <inheritdoc />
    [DataField]
    public float ReferenceDistance { get; set; }

    /// <inheritdoc />
    public float Occlusion { get; set; }

    /// <inheritdoc />
    public float PlaybackPosition { get; set; }

    /// <inheritdoc />
    public Vector2 Velocity
    {
        get => new(Velocity3.X, Velocity3.Y);
        set { if (float.IsFinite(value.X) && float.IsFinite(value.Y)) Velocity3 = new Vector3(value, Velocity3.Z); }
    }
    private Vector3 _velocity3;
    public Vector3 Velocity3 { get => _velocity3; set { RequireFinite(value); _velocity3 = value; } }

    private static void RequireFinite(Vector3 value)
    {
        if (!float.IsFinite(value.X) || !float.IsFinite(value.Y) || !float.IsFinite(value.Z))
            throw new ArgumentOutOfRangeException(nameof(value));
    }

    public void SetAuxiliary(IAuxiliaryAudio? audio)
    {
    }
}
