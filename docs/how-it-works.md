# How the proxy works

## Why this is not a simple repeater

Version 0.1 of the Python relay this project started from retransmitted frames as they came in. That cannot work on this network, whatever the radio parameters:

| Protocol fact | Consequence |
|---|---|
| A sender expects its Enhanced ACK 5 to 8 ms after its frame | Repeating a 60-byte frame alone takes 11 ms, so the far end's ACK can never arrive in time |
| Valves sleep and sample the channel once per second (CSL) | They hear nothing but a wake-up train aimed at that sample |
| A wake-up train is a 3.4 ms frame every 4.0 ms | A half-duplex radio cannot receive and repeat that, and each frame's rendezvous time goes stale |
| A retry has the same sequence number but a new frame counter | A hash over the whole frame never recognises a retry |

So the proxy terminates each hop:

```
Valve                     Proxy                       Bridge
  |--- Data ---------------->|                           |
  |<-- Enhanced ACK (5 ms) --|                           |
  |                          |--- Data, same bytes ----->|
  |                          |<-- Enhanced ACK ----------|
  |                          |                           |
  |                          |<-- wake-up train, Data ---|
  |                          |--- Enhanced ACK (5 ms) -->|
  |<-- own train, then Data -|                           |
  |--- Enhanced ACK -------->|                           |
```

Data frames travel byte for byte and stay encrypted. An Enhanced ACK has a checksum but no cryptographic tag, which is why no key is needed.

An ACK from the proxy is a promise: the sender stops trying. So the proxy gives one only for valves you named, and only if it has the queue space and the duty-cycle budget to deliver. Otherwise it stays silent and the sender retries by itself.

## Two nodes

One radio on one channel cannot be full duplex. A CC1101 either sends or receives, and a second CC1101 next to it does not help: at about +11 dBm its transmission swamps a receiver that listens for a -90 dBm valve on the same frequency.

Two nodes do give the benefit. One sits near the bridge and one near the far valves, linked over your home network. Each radio is half-duplex, but the pair serves both sides at once: while the valve node sends a one-second wake-up train, the bridge node still hears and answers the bridge. With one node, the bridge has to retry in that case. A test in the simulation shows the difference.

The pair also measures better than one node. Each node counts a frame as answered only if it hears the answer, and the answer comes from the other end of the link: the bridge node hears the bridge well and a far valve badly, the valve node the other way round. So each node sends the other the ACKs it heard, a few times a second, and holds a frame without an answer for 3 seconds in case the other node heard it. A frame counts as answered if either node heard the answer.

## Deduplication

Two kinds of repeat get different answers:

- An **echo** has exactly the bytes of a frame this node already handled: its own transmission, or one from the other node of a pair. It gets no reaction, which prevents loops. A node remembers the frames it handed to the other node for five minutes, because that node may send one much later. An ACK to such a frame would tell the other node that it arrived at the valve or the bridge.
- A **retransmission** is a sender trying again because it missed an ACK. It gets a new ACK, but the frame is not delivered twice.

A retry can be recognised without the key. It reuses the CCM nonce, so its ciphertext differs only at the frame counter, the checksum and the tag. The firmware compares the rest.

## Radio parameters

These come from [freedo](https://github.com/CappyT/freedo), a local coordinator that drives real V3+ devices, and differ from what version 0.1 of that Python relay published:

| Parameter | Python relay 0.1 | Actual |
|---|---|---|
| Carrier | 868.300 MHz | 863.125 MHz + channel x 199.951 kHz, usually channel 26 (868.324 MHz) |
| Data rate | 38.4 kBaud | 49.99 kBaud |
| Deviation | 20 kHz | 25.39 kHz |
| Sync word | `0x91D3` | `0xD391`, the chip default |
| Hardware CRC | Off | On |
| MAC | Plain 802.15.4 | 802.15.4-2015 profile with CSL, 1 s sample period |

The register values are in [phy.h](../firmware/lib/tado_gado_core/src/phy.h), and a test recomputes these numbers from them.

## What is not known yet

- **Whether the radio parameters hold for V3.** They were measured on V3+ with the same IB01 bridge. **Verified ACKs** on the dashboard answers this.
- **How long a valve waits for its ACK.** Nothing published says. The proxy answers at 5 ms, the fastest time seen on air.
- **How the bridge and the valves respond to a proxy.** A valve's phase and the bridge's phase are carried over as well as is known, and freedo's experience shows valves accept such ACKs. A proxied link was never tried.

## The simulation

[firmware/test/test_sim](../firmware/test/test_sim) runs the real proxy engine in a discrete-time model of a Tado network: a bridge, sleeping valves and one or two proxy nodes, with a table of who hears whom. A frame arrives only at a node that listened for all of it, with no other audible frame overlapping it. It shows that the logic fits the protocol as documented. It proves nothing about real radio behaviour.
