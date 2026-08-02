# RFID

Fantasi supports RFID on every device it runs on. A capable RFID frontend is one of the
criteria for a board becoming a Fantasi target. The Proxmark3, the Flipper Zero (and
Kiisu) and the Chameleon Ultra all take the same commands and write the same dump files,
so a card read on one can be emulated from another.

## Devices and frontends

Each platform carries its own driver in `platforms/*/rfid.c`. They meet at a logical
HAL (`hal/hal_rfid.h`): HF is only a framed transceive, LF is a raw acquire plus
modulate. Everything above that line (anticollision, EM4100 decode, CRC, Crypto1) is
written once in `core/rfid/` and shared. Platforms advertise their abilities through
capability bits.

| Device | HF frontend | LF | Emulation |
|---|---|---|---|
| **Proxmark3** | Xilinx Spartan-II XC2S30 FPGA driven by AT91SAM7S512 |
| **Flipper / Kiisu** | ST25R3916 in transparent mode |
| **Chameleon Ultra** | nRF52840 NFCT, MFRC522 |

NB: On the Proxmark3, FPGA bitstreams are stored compressed under `/fpga` to conserve
internal flash. They stream from the host the first time a band is used.

## Quickstart

`rfid` is a command inside the Fantasi host CLI. Running it launches the on-device app
and drops you into an interactive `rfid>` prompt with history and TAB completion:

```
fantasi> rfid
rfid> read mfc
rfid> read t5577
rfid> exit
```

For scripting, pass a single command with `-c` and it runs once and exits:

```
fantasi --usb -c "rfid read mfc"
fantasi --usb -c "rfid emulate lf-t55xx-1A2B3C4D-dump.json"
```

Example:

`read mfc` reads a MIFARE Classic card end to end: it collects nonces, recovers keys
against a dictionary, then auths and dumps every block. `read t5577` gives you a
complete dump of an LF T5577; it decodes the config block to learn how many data
blocks to expect. A bare `read <proto>` reads all blocks, and `read <proto> -b <n>`
reads one. Type `list` any time to see the protocol tree, or `help` for the verbs.

## Commands

The verbs are `search` (scan for tags), `read`, `write`, `sniff`, `raw`, `collect`,
`emulate`, `list`, `trace`, `field`, and `exit`. `read`, `write`, `list`, `collect`
and `emulate` are answered on the host; the rest dispatch to the device. `write` takes
`-b <block> -d <hex>`. `raw` accepts `-c` (append CRC), `-k` (keep the field on) and
`-s` (select the card first). `field on|off|status` drives the reader carrier directly.

## Protocols

`list` presents a band > tech > protocol tree.

The three available bands are: **LF**, **HF**, and **UHF**, in order.

## Sniffing

Every device can passively watch a live reader <-> card exchange with its own field off.
Start it with `sniff <protocol>` (e.g. `sniff mfc`) and traffic scrolls as it happens.
Press any key or `^C` (Control-C) to stop.

Each captured frame is annotated. The host recognises the protocol you named and labels
the commands inline (`SELECT`, `ANTICOLL`, `READ blk 4`, `AUTH-A blk 7`, `WRITE page 6`,
and so on), turning a hex trace into something readable. Bad CRCs and parity errors are
flagged in place. For MIFARE Classic, mfkey64 runs inline against the trace: a captured
authentication is enough to recover the sector key, with no separate cracking step.

Where the frontend can measure it, a capture opens with a coupling reading that grades
the sniff quality as **strong**, **fair** or **weak** (green, yellow, red). On the
Flipper this comes from the ST25R3916's External Field Detector.

That same measurement feeds an adaptive first-stage gain, so a reader stays in range as
its distance drifts; without it the signal would degrade up close and fade out further
away. A device that cannot sense field strength reports the coupling as *unknown*.

## Emulation

Every platform can emulate as well as read. `emulate <dump.json>` plays a saved card
back. Today that means MIFARE Classic 1K, load-modulated through a full reader
authentication and its block reads, and it runs until you press a key or `^C`.
Throughput is well ahead of the initial firmware on each device and close to a real
card's FDT (frame delay time). See the [Chameleon Ultra technical whitepaper](https://github.com/RfidResearchGroup/ChameleonUltra/wiki/technical_whitepaper).

`read -s` writes dump files when you read a card. It stays close enough to Proxmark3's
own format that dumps interchange for the common cases.

## Modularity

The app itself carries almost no protocol code. Each feature is a separate ELF module
built from the same folder: the HF reader, the LF reader, the sniffer, raw transceive,
T5577 read/write, and the MIFARE collect, read and emulate stages. A module is fetched
from the host over the protobuf link the moment it is needed, run, then deleted, so only
one is ever resident. That discipline fits the whole subsystem inside the tight flash
and heap of the smallest target, the Proxmark3.
