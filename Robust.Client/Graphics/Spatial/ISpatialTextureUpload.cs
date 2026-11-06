using System;
using Robust.Shared.Graphics;

namespace Robust.Client.Graphics.Spatial;

/// <summary>Owns one immutable material texture, including its bounded staged upload.</summary>
[NotContentImplementable]
public interface ISpatialTextureUpload : IDisposable
{
    long AllocatedBytes { get; }
    long UploadedBytes { get; }
    bool IsReady { get; }
    /// <summary>Available only after every mip level has been uploaded; remains owned until Dispose.</summary>
    Texture Texture { get; }
    /// <summary>Upload at most maximumBytes, in complete RGBA pixels. Requires at least four bytes.</summary>
    int Upload(int maximumBytes);
}
