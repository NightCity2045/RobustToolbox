using Robust.Client.Spatial;
using Robust.Shared.Network;
using Robust.Shared.Spatial.Networking;

namespace Robust.Client.GameObjects;

public sealed partial class ClientEntityManager
{
    // Net message registration lives for the connection manager's lifetime.
    // Entity systems are destroyed and recreated on disconnect/reconnect.
    private void SetupSpatialNetworking()
    {
        _networkManager.RegisterNetMessage<MsgSpatialGeometryManifest>(message =>
        {
            if (TrySystem<SpatialGeometryStreamingSystem>(out var streaming)) streaming.OnManifest(message);
        }, NetMessageAccept.Client);
        _networkManager.RegisterNetMessage<MsgSpatialGeometryFragment>(message =>
        {
            if (TrySystem<SpatialGeometryStreamingSystem>(out var streaming)) streaming.OnFragment(message);
        }, NetMessageAccept.Client);
        _networkManager.RegisterNetMessage<MsgSpatialGeometryAck>(accept: NetMessageAccept.Server);
        _networkManager.RegisterNetMessage<MsgSpatialGeometryProgress>(accept: NetMessageAccept.Server);
        _networkManager.RegisterNetMessage<MsgSpatialGeometryRequestBaseline>(accept: NetMessageAccept.Server);
    }
}
