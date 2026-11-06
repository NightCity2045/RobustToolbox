# Spatial connection identity

The fork identifies itself as `ss14-spatial-295` before authentication/world
loading. Its handshake stamp contains a magic marker, wire revision and SHA-256
fingerprint of `Robust.Shared/Spatial/spatial-protocol.json`. The descriptor records
map/transform/world/geometry and geometry-wire schemas and the pinned physics backend/version. Runtime
initialization verifies the descriptor against the compiled constants and backend
assembly. Upstream's engine version alone is not a compatibility check.

Login-start, authentication challenge and login-success carry the stamp. A server
rejects a missing/different stamp before creating a game channel. A client checks
the challenge before authentication/cryptography and checks success before channel
creation. Ordinary 295 packets, including their login-success padding, are rejected
with an explicit compatibility reason. Existing serializer and resource handshakes
still run after this check.

Fingerprint canonicalization sorts descriptor keys and hashes `key=value` lines in
UTF-8, independent of JSON indentation, property order and checkout line endings.
The standalone engine manifest includes both the descriptor and its fingerprint,
alongside the exact engine commit and file hashes. Update the wire revision for
incompatible wire changes and the descriptor when schemas/backend change; a backend
package mismatch is rejected by packaging as well as runtime initialization.

Tests exercise truncated/legacy stamps, schema/backend mismatch, both sides of
actual UDP handshakes and a matching connection through the real socket transport
and mapped-string serializer. They do not use the integration transport shortcut
for these handshake checks. Wire revision 2 adds the frame lifetime/revision stamp
and acknowledged semantic geometry transport; see
[spatial-streaming.md](spatial-streaming.md). Local prediction and history remain
separate integrations.
