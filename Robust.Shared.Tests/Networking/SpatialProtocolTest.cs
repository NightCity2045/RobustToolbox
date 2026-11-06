using System.Text.Json;
using Lidgren.Network;
using NUnit.Framework;
using Robust.Shared.Spatial;

namespace Robust.Shared.Tests.Networking;

[TestFixture]
public sealed class SpatialProtocolTest
{
    [TestCase(0)]
    [TestCase(1)]
    [TestCase(3)]
    public void HandshakeHeaderRoundTripsAtEveryExistingBitAlignment(int prefixBits)
    {
        var buffer = new NetBuffer();
        for (var i = 0; i < prefixBits; i++) buffer.Write(true);
        SpatialProtocol.Write(buffer, SpatialProtocol.Current);
        for (var i = 0; i < prefixBits; i++) Assert.That(buffer.ReadBoolean(), Is.True);
        var read = SpatialProtocol.Read(buffer);
        Assert.That(read, Is.EqualTo(SpatialProtocol.Current));
        Assert.That(SpatialProtocol.Rejection(read), Is.Null);
    }

    [Test]
    public void LegacyPaddingTruncationAndMismatchedPhysicsCannotPass()
    {
        var legacy = new NetBuffer();
        legacy.Write(new byte[100]); // Upstream login-success padding is not a valid stamp.
        Assert.That(SpatialProtocol.Rejection(SpatialProtocol.Read(legacy)), Does.Contain("ordinary Robust 295"));
        var valid = new NetBuffer();
        SpatialProtocol.Write(valid, SpatialProtocol.Current);
        for (var bytes = 0; bytes < valid.LengthBytes; bytes++)
        {
            var truncated = new NetBuffer { Data = valid.Data.ToArray(), LengthBytes = bytes };
            Assert.That(SpatialProtocol.Rejection(SpatialProtocol.Read(truncated)), Is.Not.Null);
        }
        Assert.That(SpatialProtocol.Rejection(SpatialProtocol.Current with { Revision = SpatialProtocol.Revision + 1 }),
            Does.Contain("protocol revision"));
        Assert.That(SpatialProtocol.Rejection(SpatialProtocol.Current with { Fingerprint = new string('0', 64) }),
            Does.Contain("schema/physics fingerprint"));
    }

    [Test]
    public void FingerprintIgnoresPlatformLineEndingsButChangesWithBackendVersion()
    {
        using var a = JsonDocument.Parse("{\"physics_version\":\"2.4.0\",\"wire_revision\":1}");
        using var b = JsonDocument.Parse("{\r\n  \"wire_revision\": 1,\r\n  \"physics_version\": \"2.4.0\"\r\n}");
        using var c = JsonDocument.Parse("{\"physics_version\":\"2.5.0\",\"wire_revision\":1}");
        Assert.That(SpatialProtocol.HashDescription(a.RootElement), Is.EqualTo(SpatialProtocol.HashDescription(b.RootElement)));
        Assert.That(SpatialProtocol.HashDescription(a.RootElement), Is.Not.EqualTo(SpatialProtocol.HashDescription(c.RootElement)));
        Assert.That(SpatialProtocol.Fingerprint, Has.Length.EqualTo(64));
    }
}
