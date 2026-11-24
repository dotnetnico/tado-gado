# Tado-gado -- 868 MHz Link Monitor and Proxy for Tado V3/V3+

Tado V3 and V3+ radiator valves talk to one Internet Bridge over 868 MHz. The bridge has no mesh and Tado sells no range extender, so in a large house some valves keep dropping off.

Tado-gado runs on a small ESP32 with a CC1101 radio. It measures those radio links and shows on a web page which valves really have a problem. For the valves you name, it can also act as a proxy that carries their frames to and from the bridge.

> **Status: experimental.** It runs on two ESP32-S3 boards in one house, as a pair that proxies three valves. It has not been tried in other houses, and a single node has not proxied on real hardware yet. Proxy mode transmits on your heating network, so use it with care.
>
> Tado-gado is an independent hobby project. It is not affiliated with or endorsed by tado GmbH. tado° is a trademark of tado GmbH.

| Part | State |
|---|---|
| ESP32 firmware, monitor | Runs on hardware |
| ESP32 firmware, proxy (one node) | Passes the protocol simulation. Not yet run on hardware |
| ESP32 firmware, two nodes | Runs on hardware, and passes the protocol simulation |
[docs/how-it-works.md](docs/how-it-works.md) explains why a plain repeater cannot work on this network, and how the proxy gets around that.

**The device always starts in monitor mode, where it transmits nothing.** Proxy mode is switched on from the dashboard with your admin token, and is not remembered: a restart or power cut returns to monitor, so pulling the plug is a reliable off switch. After a power cut you have to switch proxy mode on again. The list of valves you named is remembered.

## Hardware

- **ESP32-S3-DevKitC-1**, or another dual-core ESP32-S3 with the pins changed in [board.h](firmware/src/board.h). A single-core ESP32-C3 is not suitable. There are two firmware images, one per kind of flash:

  | Board | Module | Image |
  |---|---|---|
  | ESP32-S3-DevKitC-1-N8, -N16, -N8R8 and other quad flash variants | ESP32-S3-WROOM-1 | `tado-gado-esp32s3-<version>-factory.bin` |
  | ESP32-S3-DevKitC-1-N32R16V, -N16R16V and other octal flash variants | ESP32-S3-WROOM-2 | `tado-gado-esp32s3octal-<version>-factory.bin` |

  The wrong image does not boot: the board restarts over and over. Flashing the right one fixes that. The module name is printed on its metal can.
- **CC1101 module for 868 MHz**, with an 868 MHz antenna. Many cheap modules and antennas are for 433 MHz, so check before you buy.

| CC1101 | ESP32-S3 |
|---|---|
| VCC | 3V3 (never 5V, as the CC1101 is not 5 V tolerant) |
| GND | GND |
| SCK | GPIO 12 |
| MISO / SO | GPIO 13 |
| MOSI / SI | GPIO 11 |
| CSN | GPIO 10 |
| GDO0 | GPIO 4 |
| GDO2 | GPIO 5 |

## Flashing

No toolchain is needed. Download the factory image for your board from the [releases](../../releases), and flash it at offset 0 through the USB port marked UART:

```bash
pip install esptool
esptool.py --chip esp32s3 write_flash 0x0 tado-gado-esp32s3octal-v0.11.0-factory.bin
```

That example is for an octal flash board such as the N32R16V. The images from v0.5.0 and earlier are for quad flash only. Each release has a `SHA256SUMS` file to check the download against.

If esptool finds no serial port on Windows, the driver for the CP2102N chip behind the UART port is probably missing. The other port, marked USB, needs no driver: hold BOOT, press RESET, let go of BOOT, and flash through that port. Press RESET once more when it is done.

The factory image also erases the Wi-Fi details, the token and the names.

### Updating

From v0.8.0 on, a node is updated from its own dashboard, without a cable:

1. Download the `-app.bin` file for your board from the [releases](../../releases). The **Firmware** panel names it.
2. Switch to monitor, and enter the admin token.
3. Choose the file under **Firmware**, and press **Install**. It takes about a minute, and the settings stay.
4. Do the same on the other node of a pair. Both must run the same version.

The node refuses a damaged file, the factory image, and an image for the other kind of board. The old firmware stays in the other half of the flash. If the new one crashes before it has served the dashboard for 30 seconds, the old one starts again. If it hangs, or never reaches the Wi-Fi, unplug the node and plug it in again.

The file is not signed, and the dashboard uses plain HTTP. Anyone who knows the admin token, or reads it off your network while you use it, can install other firmware. Give this network a good password, and do not forward the port of the node to the internet.

To update over USB and keep the settings, flash the `-app.bin` image at `0x10000`. After an update from the dashboard, also erase the choice between the two halves, or the board may keep starting the other one:

```bash
esptool.py --chip esp32s3 erase_region 0xe000 0x2000
```

### Checking the parts

- **CC1101 module:** it must be made for 868 MHz. Modules for 433 MHz have other filter parts and do not work on 868 MHz. The register values assume a 26 MHz crystal, which most modules use; check the marking on the crystal. A 27 MHz crystal puts every frequency about 4% off, and the node hears nothing.
- **Antenna:** use one for 868 MHz, and never switch to proxy mode without an antenna: transmitting into an open output can damage the chip.
- **Pins:** the CC1101 pins in the table are free on every DevKitC-1 variant. On octal flash boards GPIO 35, 36 and 37 belong to the flash and PSRAM; the firmware does not use them.

## Getting started

1. **Connect to Wi-Fi.** On first boot the board opens a network called `tado-gado-setup` for 5 minutes, then restarts and tries again. Join it, and enter your Wi-Fi details and an admin token. The page suggests a random token: write it down, or enter your own of at least 16 characters. A shorter one is not accepted, and the dashboard then stays read-only.

   The setup network is open and has no password, so a nearby person could join it too. It only appears while no Wi-Fi is saved: once the board is set up it never opens again, even when your router is down, so nobody can use it later to change the token. To change the Wi-Fi or the token, press RESET and let go, then within 15 seconds press BOOT, hold it for about 5 seconds and let go. That erases all settings and opens the setup network once more. Do not hold BOOT while you press RESET or plug in the board: it then waits for a flash instead of starting.
2. **Open the dashboard** at `http://tado-gado-xxxx.local/`, where `xxxx` is the end of the MAC address, at the IP address that your router or the serial port shows, or at `tado-gado-xxxx` with your router's own domain, such as `tado-gado-xxxx.lan`. The board answers to no other names, which keeps web pages from reaching it through DNS rebinding.
3. **Check the radio.** The Radio panel must show a CC1101 version. If it says no CC1101 was found, check the wiring.
4. **Find the channel.** Place the board near the bridge, enter your admin token, and start **Run survey**. Change a temperature in the Tado app while it runs. It takes 5 minutes. Then press **Apply best**.
5. **Prove that it hears Tado.** **Verified ACKs** must rise above 0. Those frames carry a checksum that noise cannot produce, so this proves the radio settings fit your system. This matters most on V3, which nobody has tested. If it stays at 0, stop here.
6. **Name your valves.** The dashboard knows a valve by its address. Press **Find valve**, then send Hello to one valve in the Tado app. The valve that gets a message from the bridge shows up, and **Name** stores a name for it. Do this one valve at a time, on a quiet moment. Names survive a restart. The dashboard cannot send Hello itself: the command is encrypted, and the device has no network key.
7. **Measure for a day.** Valves with a low **Uplink ACK** or **Downlink ACK** rate, or many **Retries**, are the ones with a problem. If they all look fine, the problem is not range. See [Reading the measurements](#reading-the-measurements).

For a valve the bridge cannot hear, **Uplink ACK** only means something when the board itself hears both the valve and the bridge, so place it between them.

8. **Try the proxy on one valve.** Only do this after step 5 worked. Enter your admin token, set the **Proxy** of one weak valve to **To the valve only** if only its **Downlink ACK** is poor, **To the bridge only** if only its **Uplink ACK** is poor, or **Both ways** if both are, and press **Switch to proxy**. Watch **ACK latency** (it must stay under 8 ms), **Deliveries failed**, and the valve in the Tado app. **Switch to monitor** stops all transmissions at once.

## Reading the measurements

The device table counts over the **Last hour**, the **Last 24 hours**, or **Since start**. The hour and the day start again when the node switches between monitor and proxy, so they never mix the two. The line above the table says what the counts cover.

- **Uplink ACK:** the share of a valve's frames that the bridge answered. Low means the bridge does not hear the valve.
- **Downlink ACK:** the share of the bridge's frames that the valve answered. Below it, the same split by the wake-up train in front of the frame. A **short** train means the bridge knew when the valve listens; a **long** one means it did not. Misses after short trains point to timing, which the proxy can help with. Misses after long trains as well point to reception at the valve.
- **Trains for it:** the wake-up trains for the valve that this node heard, in seconds.
- **Tado bridge on air:** what this node heard the bridge send, frames and wake-up trains, as a share of the time. The band allows 1%. The node misses some frames, so the bridge sent at least this much.
- **Had it** (valve node only): frames the node had to deliver that the valve had already answered to the bridge itself. A high count means the valve reaches the bridge often enough by itself. The bridge node answers the bridge for the valve too, with a frame that looks the same, so the valve node tells the two apart by strength alone. It learns how strongly it hears the bridge node, shown in the pair panel, and shows a dash for a valve it hears within 12 dB of that: there it cannot tell.
- Devices heard once or twice, long ago, and never in traffic with the bridge are hidden. **Show rarely heard devices** shows them.

Only proxy valves the bridge cannot hear. If the bridge hears one too, its ACK and the proxy's ACK collide. The proxy serves at most 16 valves, though the duty cycle below will usually limit it first. The survey only runs in monitor mode.

## Two nodes

One radio cannot send and receive at once, so one node is deaf during its own transmissions. Two nodes split the work, and each has its own duty-cycle budget.

| Node mode | Stands | Does |
|---|---|---|
| **monitor** | Anywhere | Listens only. Every node starts here |
| **bridge** | Where it hears the bridge well | Answers the bridge, and delivers frames from the valves to it |
| **valve** | Near the weak valves | Answers the valves, and wakes them to deliver frames from the bridge |

Flash both boards and give them **the same admin token, on the same Wi-Fi network**. They find each other, and the node that hears the bridge best becomes the bridge node. A node that does not hear the bridge at all becomes the valve node, and learns the address of the bridge from its partner. The **Second node** panel shows what they decided. Roles do not change while proxying.

Set the valves to proxy on either node, and the list goes to the other one. **Switch to proxy** on either node starts both, and **Switch to monitor** stops both.

A valve can be proxied **Both ways**, **To the valve only** or **To the bridge only**. **To the valve only** sends just the bridge's frames through the pair, and the valve's own frames go to the bridge directly. That saves airtime, and the two ACKs to the valve no longer collide. Use it for a valve whose **Uplink ACK** is good. **To the bridge only** is the reverse, for a valve that hears the bridge well but is not heard by it. Check that in monitor mode first: a good **Downlink ACK** with a poor **Uplink ACK**.

The nodes share what they measure. Each tells the other which answers it heard, and a frame counts as answered when either node heard the answer: the bridge node hears the bridge, and the valve node hears the valves near it. They also share the names of the valves. A node keeps its own name for a valve, and a name you set or clear on one node replaces it on the other. Each node repeats its names every few seconds, so a name lost on the network still arrives.

**RSSI**, **Last seen** and **Verified ACKs** leave out what the other node sends. While a pair proxies a valve, the ACKs to and from that valve count for none of them, because either node may have sent them.

Choosing roles is automatic, but transmitting is not: a pair never starts by itself.

- If the link between the nodes drops, both stop answering until it returns, because alone a node cannot deliver what it promised.
- If either node restarts, both go back to monitor, so pulling a plug still stops everything.
- Frames travel between the nodes over UDP on your network, signed with a key derived from the admin token. A frame lost on the network is gone, and the sender got an ACK for it, so wired Ethernet or strong Wi-Fi matters. Some routers block traffic between clients, and then the nodes will not find each other.
- Each node tells the other how much duty-cycle budget and queue space it has left, so neither answers a frame the other cannot deliver.
- Both nodes must run the same version: v0.7.0 and v0.9.0 changed the messages between them, v0.10.0 added a proxy direction that an earlier node reads as both ways, and v0.12.0 changed the key that signs them. A node rejects messages from an earlier start of the other node, so after erasing the flash of one node with esptool, restart the other one too.

Keep walls or a floor between the nodes. They share the channel with the bridge and the valves, so two nodes in one room only get in each other's way.

## Limits of the proxy

- **Duty cycle:** the band allows 1%. The firmware stops at 0.9% (32.4 s per rolling hour). When that runs out it stops answering, and the valves fall back to their own retries. About 300 proxied commands per hour fill the budget of one node, so it suits a few weak valves, not a whole house. In a pair the valve node spends most of it, and the bridge node answers from its own budget.
- **Where the budget goes:** **Airtime, last hour** in the device table shows what this node sent for each valve, and the share of its budget. A valve that takes a lot and reaches the bridge by itself is better left off the proxy.
- **Deaf while sending:** the node hears nothing during its own wake-up train, which takes up to 1 s. Senders retry by themselves.
- **Output power:** the default of about +11 dBm is close to the 25 mW e.r.p. limit of the band. Lower it for an antenna with gain, for example with `POST /api/radio` and `{"pa": 96}`. The value is a CC1101 PATABLE entry; 192 is the default.
- **Unknowns:** nothing published says how long a valve waits for its ACK, and a proxy was never tried on a real bridge.

## Before you build

Moving the bridge may solve the problem for less. It needs only USB power and Ethernet, so a long network cable or a powerline adapter can bring it to the centre of the house.

## Repository

| Path | Contents |
|---|---|
| `firmware/` | ESP32 firmware (C++, PlatformIO) |
| `firmware/lib/tado_gado_core/` | Hardware-independent core, tested on the build machine |
| `firmware/test/test_core/` | Unit tests of the core |
| `firmware/test/test_sim/` | The proxy engine in a simulated Tado network |
| `firmware/web/dashboard.html` | The dashboard, embedded in the firmware at build time |
| `docs/` | How the proxy works |
| `VERSION` | The version of the firmware and of a release |

## Development

Install the commit hooks once. They format and check the code before each commit, with the same tools and versions as CI:

```bash
pip install pre-commit
pre-commit install
```

Install [PlatformIO](https://platformio.org/install/cli) for the firmware:

```bash
cd firmware
pio test -e native
pio run -e esp32-s3
pio run -e esp32-s3 -t upload
```

## Releases

Pushing a tag such as `v0.7.0`, equal to the version in `VERSION`, runs all tests and creates a GitHub release with the firmware images.

## Credits

The radio register set, frame layouts and protocol timing come from the [freedo](https://github.com/CappyT/freedo) project (Apache-2.0), whose `docs/PROTOCOL.md` is based on on-air captures. No freedo code is included.

The project started from an MIT-licensed Python relay by Bojan Belic (Sigma Delta Software). None of its code is left.

## License

MIT, see [LICENSE](LICENSE).
