# RT <-> Compute API

The real-time processor and compute platform communicate via Protobuf messages over a UART connection.

```{note}
**Why not gRPC?**
gRPC isn't natively supported by Zephyr. Further, the C++ libraries required for gRPC are huge, and would be unlikely to fit on the RT processor. It's also quite heavy for simple command/telemetry interactions.

**Why not embedded-focused RPC frameworks?**
Simply put, I'm not very familiar with them. For now, a simple custom protocol built on Protobuf is sufficient.
```

## Message Schema

The message schema is defined using Protobuf. The Protobuf definitions are located in the [`proto/rt_compute_api/`](https://github.com/sailorbob134280/benji/tree/main/proto) directory of the main repository.

Note that at this stage of the project, the message schema is still evolving. Not all features are supported yet, and breaking changes may occur.

## Interaction Model

The interaction model is a simple controller-peripheral pattern, with the compute platform acting as the controller and the real-time processor as the peripheral. In this model, the RT core does not speak unless spoken to, even for telemetry, which must be rerquested by the compute platform.

There are two primary interaction types:

- **Command** - The compute platform sends a command message to the real-time processor to perform an action, and the real-time processor responds with a status message indicating either success or an error message.
- **Request/Response** - The compute platform sends a request message to the real-time processor indicating which data it would like to receive, and the real-time processor responds with a either the requested data or an error message.

The RT core can only service one interacttion at a time. Interaction completion is indicated by the response message from the RT core (status for commands, responses or error message for requests). For simplicity, streaming interactions (client, server, or bidirectional) are not currently supported.

```{success}
Looking towards the future, eventually it will be worth either leveraging an existing embedded RPC framework or building a `protoc` plugin that will generate client and server stubs.
```

## Framing

Each Protobuf message is framed with a highly simple custom protocol to allow for message boundary detection and error checking. The framing format is as follows:

- **Sync Marker**: 4 bytes, fixed value of `0x35 0x2e 0xf8 0x53` (CCSDS embedded sync marker)
- **Message Length**: 4 bytes, unsigned little-endian integer indicating the length of the Protobuf message in bytes, not including the sync marker, length field, or checksum.
- **Data Field**: Variable length Protobuf-encoded message.
- **Checksum**: 2 bytes, CRC16 checksum of the Data Field.

This framing format allows the receiver to detect message boundaries by searching for the sync marker, read the length of the incoming message, and verify data integrity using the checksum.

```{note}
This framing format is intentionally simple. Others (such as COBS or SLIP) were considered, but for now they're simply overkill, and this is very easy to implement on both ends.
```
