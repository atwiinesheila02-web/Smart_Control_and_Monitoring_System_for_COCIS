# CoCIS SmartGuard

ESP-IDF coursework simulation for one ESP32-S3, two monitored rooms, separate
entry/exit RFID readers, passage confirmation, lighting and emergency evacuation.

## Build and run

The project was built with ESP-IDF v6.0.2 for `esp32s3`.
In an ESP-IDF terminal, run:

```text
idf.py -B build-current reconfigure build
```

Stop and restart Wokwi after rebuilding. `wokwi.toml` and the VS Code workspace
settings use `build-current`; the older `build` directory is not the active
firmware. Do not run `wokwi-cli init` over the supplied diagram and configuration.

Wokwi CLI requires a locally configured `WOKWI_CLI_TOKEN`. Do not put it in source
files. The CLI can automate signals and capture screenshots; a signal measurement
does not verify audible playback through the computer's speakers.

shared.h contains types, public declarations and externs. Configuration values remain in main.c; there is no config.h or system.h. Each source has section banners. The earlier module names in notes identify sections inside these merged files. The labelled diagram.json is included.

| File | Responsibility |
|---|---|
| `main/main.c` | Configuration, startup, shared state, GPIO helpers and clock |
| `main/access.c` | RFID, access policy, room coordinator, schedule, keypad and serial commands |
| `main/passage.c` | Passage journal, debounced entry/exit verification, servos, LEDs and buzzer |
| `main/rooms.c` | Software I2C, four LCDs, telemetry, room PIR sequences and lighting |
| `main/safety.c` | NVS checkpoints, SD logging, emergency latch and final-exit checklist |
| `main/shared.h` | Shared types, declarations and exported configuration |

Build all five C files together. Older split modules must not be added alongside
these merged sources. No separate presenter or setup files are required.

## Normal entry and exit

1. During opening hours, scan a registered card on the entry reader. Blue and
   Green presets are provided by the current simulation defaults. The door opens,
   but building occupancy does not change yet.
2. Hold the blue ENTRY PASSAGE button for about 200 ms, then release it within
   eight seconds. A stable blocked/clear cycle confirms entry and increments the
   building count exactly once.
3. To enter a monitored room, trigger its OUTER PIR and then INNER PIR within six
   seconds. The room count increases; the building count stays unchanged.
4. Wait for both PIRs to clear before a new movement. To leave the room, trigger
   INNER then OUTER within six seconds. The room count decreases; building count
   stays unchanged because the person is still inside the building.
5. Scan the same card at the exit reader, then hold/release EXIT PASSAGE. Confirmed
   passage decreases building count only. If every recorded occupant is still in
   a room, the scan is refused with `Leave room first`.

A single PIR trigger, simultaneous triggers, or a late second trigger does not
count a movement. Room PIRs are anonymous: the system tracks room totals, not
individual room identity. With multiple occupants, normal exit authorization uses
available capacity outside the monitored rooms; it cannot prove who left which
room. The internal difference between building and room counts can include toilets
and other spaces, so user output does not label it as corridor occupancy.

An unapproved passage cycle, an interrupted saved passage intent, or a state-save
failure marks counts uncertain. Do not bypass these conditions by pressing passage
buttons without an approved window. Each card must have a confirmed entry before
normal exit; a repeat entry while it is recorded inside is rejected.

## Occupancy and administrator commands

Viewing commands do not require login:

```text
occupancy
rooms
status
time
users
who
passage status
checklist
help
```

`occupancy` and `status` show Room 1, Room 2 and Building count. Building count
includes everyone recorded inside, including people outside the monitored rooms.
After an emergency these values are explicitly last-known and unverified.

Administrator examples (the pilot PIN is configured in `main/main.c`):

```text
login 2026
schedule set 05:00 21:00
time set 2026-10-31 12:00:00
adduser 12345678 Visitor
logout
```

Use the UID printed by the serial monitor for registration. Users recorded inside
cannot be deleted. Admin sessions expire after 60 seconds. The `person enter 1|2`
and `person leave 1|2` commands are simulation test hooks, not identified movement.

## Automatic lighting

Both rooms use the same rule: an occupied room switches its light on below
100 lux and off above 150 lux. Between those thresholds it retains its previous
state to prevent flicker. Empty rooms switch lights off. The LCD shows measured
lux and commanded lighting state. Live tests check the physical relay contacts
and lamp as well as the serial state.

## Closing warnings and LCDs

Default hours are 05:00-21:00; persisted admin settings take precedence. A DS1307
RTC supplies local wall-clock values; set it to Kampala time. RTC read failure
uses the ESP32 software clock fallback.

The first check in the final ten minutes produces a ten-second pulsing warning.
LCDs show `BUILDING CLOSING / IN 10 MINUTES`, then update the remaining minutes.
Temporary access messages and the entrance admin menu remain usable. At the
open-to-closed transition a second ten-second pulsing alarm sounds, and LCDs show
`BUILDING CLOSED`. Both work with overnight schedules. A warning sounds once per
closing occurrence during a run; rebooting in the warning window warns again.
Starting while already closed does not produce a closing-transition alarm.

After successful final checkout, the entrance returns to `ENTRANCE / Scan your
card`; it does not show `BUILDING CLEAR`. The exit retains the checklist result.
Actual checklist problems remain visible and are recorded in logs.

## Emergency and smoke simulation

Press the red button or move the smoke slide switch RIGHT to activate evacuation.
The exit is held open without RFID, its LED flashes, and the buzzer pulses. New
normal entries and room counting stop; existing counts remain last-known records.
Emergency output takes priority over schedule restrictions and closing warnings.

Releasing the button or moving the switch LEFT clears the input but does not
unlatch the alarm. After all inputs have been clear for at least one second:

```text
login 2026
emergency reset
```

Counts remain unverified after reset. Only after physically confirming the entire
building is empty, run `occupancy reconcile empty`. This resets building and room
records together. Do not use reconciliation just to dismiss a warning.

## Wiring summary

| Device | ESP32-S3 GPIO |
|---|---|
| RFID entry/exit chip selects | 10 / 9 |
| Shared SPI MOSI / MISO / SCK | 11 / 13 / 12 |
| Shared RFID reset / SD chip select | 7 / 6 |
| Entry/exit servos | 4 / 5 |
| Entry/exit LEDs / buzzer | 16 / 17 / 18 |
| Room 1 OUTER / INNER / LDR / relay | 1 / 15 / 2 / 14 |
| Room 2 OUTER / INNER / LDR / relay | 41 / 47 / 3 / 38 |
| LCD and RTC SDA / SCL | 39 / 40 |
| Emergency button / smoke alarm | 21 / 42 |
| Entry/exit passage buttons | 19 / 20 |

GPIO19/20 serve passage inputs, so UART0 is the console and the secondary USB
console is disabled. Four LCDs have distinct addresses 0x27, 0x26, 0x25 and 0x24;
the DS1307 is at 0x68. The labelled `diagram.json` is the normal simulation circuit.

## Validation

Run the native suite in Linux/WSL with GCC and Python 3:

```text
SMARTGUARD_SANITIZE=1 python3 tests/run_tests.py
```

It syntax-checks all five sources and runs production-logic tests for access,
occupancy, concurrent room updates, persistence failures, authentication, clocks,
lighting, emergency behavior, passage verification and RFID recovery. These use
mocked SDK/hardware calls. They do not replace an ESP32 build or live simulation.

Live scenarios are in `tests/wokwi`. With Wokwi CLI installed and the token set:

```text
wokwi-cli . --scenario tests/wokwi/system.yaml
wokwi-cli . --scenario tests/wokwi/blue-green.yaml
wokwi-cli . --diagram-file tests/wokwi/room-input-fixture.json --scenario tests/wokwi/room-sequence.yaml
wokwi-cli . --scenario tests/wokwi/lighting.yaml
wokwi-cli . --scenario tests/wokwi/lighting-room1.yaml
```

The room-input fixture substitutes pushbuttons with pull-down resistors for the
two Room 2 PIR outputs so automation can drive the real firmware GPIO sequence.
It leaves the normal project diagram unchanged. Lighting scenarios use the serial
room-entry test hook to isolate LDR/relay behavior. Screenshots and serial logs
provide evidence; `validation.json` records the latest checks and their scope.

NVS is the authoritative state checkpoint. SD files provide logs and exports;
missing SD storage produces serial backup messages. Failed NVS writes disable
normal grants and roll back uncommitted state. Flash erasure or a fresh simulator
flash image removes saved state. The smoke switch is a digital alarm simulator,
not a physical smoke measurement device.
