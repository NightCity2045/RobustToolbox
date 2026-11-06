using Lidgren.Network;
using Robust.Shared.Serialization;
using Robust.Shared.Spatial;

#nullable disable

namespace Robust.Shared.Network.Messages.Handshake
{
    internal sealed class MsgLoginSuccess : NetMessage
    {
        // Same deal as MsgLogin, helper for NetManager only.
        public override string MsgName => string.Empty;

        public override MsgGroups MsgGroup => MsgGroups.Core;

        public NetUserData UserData;
        public LoginType Type;
        public SpatialProtocolStamp Protocol = SpatialProtocol.Current;

        public override void ReadFromBuffer(NetIncomingMessage buffer, IRobustSerializer serializer)
        {
            var name = buffer.ReadString();
            var id = buffer.ReadGuid();
            var patreonTier = buffer.ReadString();
            if (patreonTier.Length == 0)
                patreonTier = null;

            UserData = new NetUserData(new NetUserId(id), name) {PatronTier = patreonTier};
            Type = (LoginType) buffer.ReadByte();
            Protocol = SpatialProtocol.Read(buffer);
        }

        public override void WriteToBuffer(NetOutgoingMessage buffer, IRobustSerializer serializer)
        {
            buffer.Write(UserData.UserName);
            buffer.Write(UserData.UserId);
            buffer.Write(UserData.PatronTier);
            buffer.Write((byte) Type);
            SpatialProtocol.Write(buffer, Protocol);
            buffer.Write(new byte[100]);
        }
    }
}
