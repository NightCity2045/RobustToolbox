using Robust.Server.Spatial;
using Robust.Shared.Network;
using Robust.Shared.Spatial.Networking;

namespace Robust.Server.GameObjects;

public sealed partial class ServerEntityManager
{
    private void SetupSpatialNetworking()
    {
        _networkManager.RegisterNetMessage<MsgSpatialGeometryManifest>(accept: NetMessageAccept.Client);
        _networkManager.RegisterNetMessage<MsgSpatialGeometryFragment>(accept: NetMessageAccept.Client);
        _networkManager.RegisterNetMessage<MsgSpatialGeometryAck>(message =>
        {
            if (TrySystem<SpatialGeometryStreamingSystem>(out var streaming)) streaming.OnAck(message);
        }, NetMessageAccept.Server);
        _networkManager.RegisterNetMessage<MsgSpatialGeometryProgress>(message =>
        {
            if (TrySystem<SpatialGeometryStreamingSystem>(out var streaming)) streaming.OnProgress(message);
        }, NetMessageAccept.Server);
        _networkManager.RegisterNetMessage<MsgSpatialGeometryRequestBaseline>(message =>
        {
            if (TrySystem<SpatialGeometryStreamingSystem>(out var streaming)) streaming.OnBaselineRequest(message);
        }, NetMessageAccept.Server);
    }
}
