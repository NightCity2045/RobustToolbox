# Engine-owned Lidgren correction

`NetConnection.cs` is the corresponding source file from the pinned
`SpaceWizards.Lidgren.Network` submodule at
`4a111ba2e392bc0adcf2b724999c40cbbaae4662`, under its
[MIT license](../Lidgren.Network/LICENSE). The engine wrapper project compiles
this file in place of that one source file. The upstream submodule is unchanged.

The only functional change is an early return from `Heartbeat` when the
connection is already disconnected. `ExecutePeerShutdown` first calls
`ExecuteDisconnect`, which resets reliable send channels, then performs a final
peer heartbeat. Previously that heartbeat consumed queued acknowledgements
against the reset channels: a null stored message threw on the network thread
outside its normal exception handler and terminated the process in debug builds.
Remote disconnect can produce the same ordering.

`LidgrenShutdownTest` deterministically exercises this sequence; the spatial
handshake integration tests also exercise actual UDP peers. Keep this correction
when updating the submodule, or remove the replacement once upstream provides it.
