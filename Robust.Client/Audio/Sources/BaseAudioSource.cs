using System;
using System.Numerics;
using OpenTK.Audio.OpenAL;
using Robust.Client.Audio.Effects;
using Robust.Shared.Audio.Effects;
using Robust.Shared.Audio.Sources;
using Robust.Shared.Audio.Systems;
using Robust.Shared.Maths;

namespace Robust.Client.Audio.Sources;

public abstract class BaseAudioSource : IAudioSource
{
    /*
     * This may look weird having all these methods here however
     * we need to handle disposing plus checking for errors hence we get this.
     */

    /// <summary>
    /// Handle to the AL source.
    /// </summary>
    protected int SourceHandle;

    /// <summary>
    /// Source to the EFX filter if applicable.
    /// </summary>
    protected int FilterHandle;

    internal readonly AudioManager Master;

    /// <summary>
    /// Prior gain that was set.
    /// </summary>
    private float _gain;

    private float _occlusion;

    private bool IsEfxSupported => Master.IsEfxSupported;

    internal BaseAudioSource(AudioManager master, int sourceHandle)
    {
        Master = master;
        SourceHandle = sourceHandle;
        AL.GetSource(SourceHandle, ALSourcef.Gain, out _gain);
    }

    /// <inheritdoc />
    public void Pause()
    {
        AL.SourcePause(SourceHandle);
    }

    /// <inheritdoc />
    public void StartPlaying()
    {
        if (Playing)
            return;

        Playing = true;
    }

    /// <inheritdoc />
    public void StopPlaying()
    {
        if (!Playing)
            return;

        Playing = false;
    }

    /// <inheritdoc />
    public void Restart()
    {
        AL.SourceRewind(SourceHandle);
        StartPlaying();
    }

    /// <inheritdoc />
    public virtual bool Playing
    {
        get
        {
            _checkDisposed();
            var state = AL.GetSource(SourceHandle, ALGetSourcei.SourceState);
            Master._checkAlError();
            return state == (int)ALSourceState.Playing;
        }
        set
        {
            _checkDisposed();

            if (value)
            {
                AL.SourcePlay(SourceHandle);
            }
            else
            {
                AL.SourceStop(SourceHandle);
            }


            Master._checkAlError();
        }
    }

    /// <inheritdoc />
    public bool Looping
    {
        get
        {
            _checkDisposed();
            AL.GetSource(SourceHandle, ALSourceb.Looping, out var ret);
            Master._checkAlError();
            return ret;
        }
        set
        {
            _checkDisposed();
            AL.Source(SourceHandle, ALSourceb.Looping, value);
            Master._checkAlError();
        }
    }

    /// <inheritdoc />
    public bool Global
    {
        get
        {
            _checkDisposed();
            AL.GetSource(SourceHandle, ALSourceb.SourceRelative, out var value);
            Master._checkAlError();
            return value;
        }
        set
        {
            _checkDisposed();
            AL.Source(SourceHandle, ALSourceb.SourceRelative, value);
            Master._checkAlError();
        }
    }

    /// <inheritdoc />
    public virtual Vector2 Position
    {
        get { var p = Position3; return new Vector2(p.X, p.Y); }
        set
        {
            _checkDisposed();
            if (!AreFinite(value.X, value.Y)) return;
            Position3 = new Vector3(value, Position3.Z);
        }
    }

    /// <inheritdoc />
    public virtual Vector3 Position3
    {
        get
        {
            _checkDisposed();
            AL.GetSource(SourceHandle, ALSource3f.Position, out var x, out var y, out var z);
            Master._checkAlError();
            return new Vector3(x, y, z);
        }
        set
        {
            _checkDisposed();

            RequireFinite(value);
            AL.Source(SourceHandle, ALSource3f.Position, value.X, value.Y, value.Z);
            Master._checkAlError();
        }
    }

    /// <inheritdoc />
    public float Pitch
    {
        get
        {
            _checkDisposed();
            AL.GetSource(SourceHandle, ALSourcef.Pitch, out var value);
            Master._checkAlError();
            return value;
        }
        set
        {
            _checkDisposed();
            AL.Source(SourceHandle, ALSourcef.Pitch, value);
            Master._checkAlError();
        }
    }

    /// <inheritdoc />
    public float Volume
    {
        get
        {
            var gain = Gain;
            var volume = SharedAudioSystem.GainToVolume(gain);
            return volume;
        }
        set => Gain = SharedAudioSystem.VolumeToGain(value);
    }

    /// <inheritdoc />
    public float Gain
    {
        get
        {
            _checkDisposed();
            AL.GetSource(SourceHandle, ALSourcef.Gain, out var gain);
            Master._checkAlError();
            return gain;
        }
        set
        {
            if (float.IsNaN(value))
            {
                Master.LogError($"Tried to set NaN gain, setting audio source to 0f: {Environment.StackTrace}");
                value = 0f;
            }

            _checkDisposed();
            var priorOcclusion = 1f;
            if (!IsEfxSupported)
            {
                AL.GetSource(SourceHandle, ALSourcef.Gain, out var priorGain);
                // Default to 0 to avoid spiking audio, just means it will be muted for a frame in this case.
                priorOcclusion = _gain == 0 ? 1f : priorGain / _gain;
            }

            _gain = value;
            AL.Source(SourceHandle, ALSourcef.Gain, _gain * priorOcclusion);
            Master.LogALError($"Gain is {_gain:0.00} and priorOcclusion is {priorOcclusion:0.00}. EFX supported: {IsEfxSupported}");
        }
    }

    /// <inheritdoc />
    public float MaxDistance
    {
        get
        {
            _checkDisposed();
            AL.GetSource(SourceHandle, ALSourcef.MaxDistance, out var value);
            Master._checkAlError();
            return value;
        }
        set
        {
            _checkDisposed();
            AL.Source(SourceHandle, ALSourcef.MaxDistance, value);
            Master.LogALError($"MaxDistance is {value:0.00}");
        }
    }

    /// <inheritdoc />
    public float RolloffFactor
    {
        get
        {
            _checkDisposed();
            AL.GetSource(SourceHandle, ALSourcef.RolloffFactor, out var value);
            Master._checkAlError();
            return value;
        }
        set
        {
            _checkDisposed();
            AL.Source(SourceHandle, ALSourcef.RolloffFactor, value);
            Master.LogALError($"RolloffFactor is {value:0.00}");
        }
    }

    /// <inheritdoc />
    public float ReferenceDistance
    {
        get
        {
            _checkDisposed();
            AL.GetSource(SourceHandle, ALSourcef.ReferenceDistance, out var value);
            Master._checkAlError();
            return value;
        }
        set
        {
            _checkDisposed();
            AL.Source(SourceHandle, ALSourcef.ReferenceDistance, value);
            Master.LogALError($"ReferenceDistance is {value:0.00}");
        }
    }

    /// <inheritdoc />
    public float Occlusion
    {
        get => _occlusion;
        set
        {
            _checkDisposed();
            var cutoff = MathF.Exp(-value * 1);
            var gain = MathF.Pow(cutoff, 0.1f);
            if (IsEfxSupported)
            {
                SetOcclusionEfx(gain, cutoff);
            }
            else
            {
                gain *= gain * gain;
                AL.Source(SourceHandle, ALSourcef.Gain, _gain * gain);
            }

            _occlusion = value;
            Master._checkAlError();
        }
    }

    /// <inheritdoc />
    public float PlaybackPosition
    {
        get
        {
            _checkDisposed();
            AL.GetSource(SourceHandle, ALSourcef.SecOffset, out var value);
            Master._checkAlError();
            return value;
        }
        set
        {
            _checkDisposed();

            value = MathF.Max(value, 0f);
            AL.Source(SourceHandle, ALSourcef.SecOffset, value);
            Master._checkAlError($"Tried to set invalid playback position of {value:0.00}");
        }
    }

    /// <inheritdoc />
    public Vector2 Velocity
    {
        get { var v = Velocity3; return new Vector2(v.X, v.Y); }
        set
        {
            _checkDisposed();
            if (!AreFinite(value.X, value.Y)) return;
            Velocity3 = new Vector3(value, Velocity3.Z);
        }
    }

    /// <inheritdoc />
    public Vector3 Velocity3
    {
        get
        {
            _checkDisposed();

            AL.GetSource(SourceHandle, ALSource3f.Velocity, out var x, out var y, out var z);
            Master._checkAlError();
            return new Vector3(x, y, z);
        }
        set
        {
            _checkDisposed();

            RequireFinite(value);
            AL.Source(SourceHandle, ALSource3f.Velocity, value.X, value.Y, value.Z);
            Master._checkAlError();
        }
    }

    void IAudioSource.SetAuxiliary(IAuxiliaryAudio? audio)
    {
        _checkDisposed();
        if (!IsEfxSupported)
            return;

        if (audio is AuxiliaryAudio impAudio)
        {
            ALC.EFX.Source(SourceHandle, EFXSourceInteger3.AuxiliarySendFilter, impAudio.Handle, 0, 0);
        }
        else
        {
            ALC.EFX.Source(SourceHandle, EFXSourceInteger3.AuxiliarySendFilter, 0, 0, 0);
        }

        Master._checkAlError();
    }

    private void SetOcclusionEfx(float gain, float cutoff)
    {
        if (FilterHandle == 0)
        {
            FilterHandle = ALC.EFX.GenFilter();
            ALC.EFX.Filter(FilterHandle, FilterInteger.FilterType, (int) FilterType.Lowpass);
        }

        ALC.EFX.Filter(FilterHandle, FilterFloat.LowpassGain, gain);
        ALC.EFX.Filter(FilterHandle, FilterFloat.LowpassGainHF, cutoff);
        AL.Source(SourceHandle, ALSourcei.EfxDirectFilter, FilterHandle);
    }

    protected static bool AreFinite(float x, float y)
    {
        if (float.IsFinite(x) && float.IsFinite(y))
        {
            return true;
        }

        return false;
    }

    protected static void RequireFinite(Vector3 value)
    {
        if (!float.IsFinite(value.X) || !float.IsFinite(value.Y) || !float.IsFinite(value.Z))
            throw new ArgumentOutOfRangeException(nameof(value), "Audio coordinates and velocities must be finite.");
    }

    ~BaseAudioSource()
    {
        Dispose(false);
    }

    public void Dispose()
    {
        Dispose(true);
        GC.SuppressFinalize(this);
    }

    protected abstract void Dispose(bool disposing);

    protected bool _isDisposed()
    {
        return SourceHandle == -1;
    }

    protected void _checkDisposed()
    {
        if (SourceHandle == -1)
        {
            throw new ObjectDisposedException(nameof(BaseAudioSource));
        }
    }
}
