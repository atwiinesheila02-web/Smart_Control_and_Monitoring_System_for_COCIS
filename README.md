# SmartGuard â€” five-file project

## Running the current passage and emergency firmware

After changing source files, run `idf.py -B build-current reconfigure build` in an ESP-IDF terminal,
then stop and restart Wokwi. `wokwi.toml` loads `build-current/cocis_smartguard.bin`;
editing C files alone does not update that firmware. The startup log must include
`Firmware: verified passage + emergency inputs` and its build date/time.

Scan an authorized entry card while the schedule is open. The door opens, but
occupancy must stay unchanged until the blue ENTRY PASSAGE button is held for
about 200 ms and released within eight seconds. Expect PASSAGE VERIFIED only
after release. GPIO19 uses an internal pull-up and the button connects it to GND.
The secondary USB console is disabled because GPIO19/20 serve passage inputs;
use the UART0 serial monitor.

Press the red EMERGENCY EXIT button (GPIO21 to GND). The exit must open without
a card, its LED must flash, and the buzzer must pulse. The alarm stays latched
after release. To end a demo, clear the smoke switch, release the button for at
least one second, then use `login 2026` and `emergency reset`. Only after checking
the building is empty, use `occupancy reconcile empty` to clear uncertain counts.

The implementation is merged into exactly five C files plus one shared header in main/:

| Presenter | File | Scope |
|---|---|---|
| 1 | main.c | Configuration, shared definitions, startup, helpers, clock |
| 2 | access.c | RFID, access rules, schedule, admin menus and CLI |
| 3 | passage.c | Verified passage, doors, LEDs and buzzer |
| 4 | rooms.c | I2C, LCDs, telemetry and room/entrance tasks |
| 5 | safety.c | State persistence, SD storage, emergency and checklist |

shared.h contains types, public declarations and externs. Configuration values remain in main.c; there is no config.h or system.h. Each source has section banners. The earlier module names in notes identify sections inside these merged files. Presenter guides are under presenters/. The labelled diagram.json is included.

Build the complete project; do not add the old source files alongside these merged versions. The root and main CMakeLists.txt files are included. Native checks pass, but target ESP-IDF compilation and Wokwi execution remain to be done.

# CoCIS SmartGuard - verified passage and closing checklist

Start with PASSAGE_CHECKLIST_SETUP.md for verified entry/exit and the last-person-out checklist. EMERGENCY_SETUP.md covers smoke/button evacuation.

This is the revised single-ESP32-S3 ESP-IDF coursework implementation. It keeps three FreeRTOS terminals (entrance, room 1 and room 2) on one ESP32. Configuration values, pins, startup and shared state definitions remain in main/main.c. No config.h or system.h is needed. This package contains code and setup notes, not the coursework report.

## Install in the existing project

1. Back up the original project.
2. Replace the contents of its main folder with the supplied main folder, including all headers and main/CMakeLists.txt. Do not leave another copy of the old main.c in the build.
3. Keep the existing Wokwi diagram.json, wokwi.toml, sdkconfig and other project files. They are not included in this package.
4. Keep the existing top-level CMakeLists.txt when integrating. A minimal top-level file is supplied for a fresh ESP-IDF project.
5. Build with the same ESP-IDF version as the original project. For a new project: idf.py set-target esp32s3, then idf.py build.
6. Run the existing Wokwi simulation using the newly built firmware. Check its firmware/ELF paths in wokwi.toml.

The standard ESP-IDF partition table must include NVS (the default normally does). The new main/CMakeLists.txt adds the nvs_flash dependency. This is ESP-IDF C code, not an Arduino sketch.

## Latest additions

- Verified passage with one active-low beam per designated doorway (GPIO19/20 by default).
- Last-person-out checklist after the final confirmed exit, plus the checklist serial command.
- See PASSAGE_CHECKLIST_SETUP.md for hardware requirements, UART0/native-USB constraints and exact demo steps.

## Earlier fixes

- Users, inside flags, entry times, room counts and access schedule are checkpointed together in an ESP32 NVS blob. Room counts and lights are reconstructed on reboot. SD card logs remain separate.
- User and schedule changes are accepted only after checkpoint saving succeeds. On a checkpoint failure, RAM is rolled back to the last saved state, both controlled doors close, grants are disabled and the status shows a state fault. NVS is never silently erased on an error.
- Both room counts are updated through a shared coordinator under a recursive mutex. A movement is committed once and then applied to the room output. The former count-changing queue request/reply exchange is no longer used, so a dropped or duplicate reply cannot alter occupancy.
- Queues remain for terminal telemetry, sensor faults, keypad events and storage queries. Queue saturation is reported. Stale telemetry cannot overwrite current room counts or lighting state. It is not a protocol for independent ESP32 boards.
- Entry, room and clock tasks share a state lock; LCD rendering, SPI, I2C, ADC and clock access retain their resource locks. SD log-display results are published under a dedicated mutex rather than a volatile ready flag alone.
- Access logs are appended synchronously under the SPI mutex with checked write/close results. They no longer pass through the queue that could drop them. Failed/unavailable SD writes produce a serial backup and increment the visible log-failure counter. This does not make an absent or failed card durable storage.
- Long storage queries release SPI between filesystem operations; printing an export does not hold the bus. SD mount errors do not automatically format the card.
- During normal operation both doors follow the schedule. The emergency latch takes priority: the exit is held open without RFID even after hours, while new entrance grants are blocked. The old manual unlock command remains disabled.
- Administrator authentication is rechecked when a tag is registered. Both interfaces prevent deleting someone who is recorded inside. Failed saves are not reported as successful changes. Serial input no longer echoes the login PIN.
- Invalid saved schedules and impossible calendar dates are rejected. RTC failure uses the software fallback clock, 12-hour RTC values are decoded, and time formatting uses a consistent local-wall-clock encoding.
- Startup checks all queue/mutex allocations and task creation. Creation calls execute even in builds where assert() is disabled.

## File guide

| Module | Purpose |
| --- | --- |
| main.c | Configuration, hardware startup, shared definitions and task creation |
| passage.c / passage.h | Debounced doorway confirmation, pending-window journal and recovery |
| checklist.c / checklist.h | Last-person-out checks and reporting |
| emergency.c / emergency.h | Smoke/button detection, latch, priority actuation, recovery and reset |
| state.c / state.h | Durable checkpoint, rollback and state synchronization |
| access.c / access.h | Users, access policy, schedule, admin and occupancy coordinator |
| terminals.c / terminals.h | Entrance and room task loops, PIR sequences, LDR and lights |
| controls.c / controls.h | Serial and keypad interfaces |
| storage.c / storage.h | Checked SD writes, query worker and LCD log publication |
| clock.c / clock.h | DS1307 and software clock |
| rfid.c / rfid.h | Both card readers |
| actuators.c / actuators.h | Door servos, LEDs and buzzer |
| display.c / display.h | Four LCDs |
| bus.c / bus.h | Non-authoritative terminal telemetry |
| i2c_bus.c / i2c_bus.h | Shared software I2C |
| helpers.c / helpers.h | GPIO, timing and BCD helpers |
| shared.h / types.h | Shared declarations and types |

## Native tests already run

Run python3 tests/run_tests.py with GCC and Python 3 on a host computer. For sanitizer checks, run SMARTGUARD_SANITIZE=1 python3 tests/run_tests.py on a shell that supports this environment-variable syntax.

- All 16 application C files passed native syntax checks using test SDK declarations.
- The 95 reliability regression assertions exercised actual application logic (scan helpers now explicitly simulate passage), including access rules, recovery, deletion persistence, failed NVS writes/commits, corrupted checkpoints, schedule boundaries, authentication, PIR order, lighting hysteresis, stuck sensors, stale telemetry, queue saturation and SD failure handling.
- An additional 83 emergency assertions passed: smoke alone, button alone, both sources, alarm/output priority, independent polling while normal state/SPI locks are held, persistence failures, reboot protection, authenticated reset, uncertainty and manual reconciliation.
- Another 73 verified-passage/checklist assertions passed: no count on scan, full-cycle confirmation, timeout, bounce rejection, reservation, ambiguous cycles, obstruction handling, restart recovery, write failures, timer wraparound and checklist failures.
- Two host threads completed 400 room transitions using the production coordinator and real host mutexes. Counts remained consistent.
- AddressSanitizer and UndefinedBehaviorSanitizer passed. Leak detection is disabled because lifetime RTOS mutex allocations are intentionally retained by the host mock.

These are native logic tests with mocked ESP-IDF/NVS/hardware calls. They are not a target ESP-IDF build, an ESP32 timing/stress test, an actual flash-power-loss test or a Wokwi demonstration. Full target compilation and simulation are still required.

## Simulation checks to perform next

1. Read a known card: the door opens, but occupancy must not change until the corresponding passage sensor is blocked and cleared. Test entry and exit separately; reject unknown cards and duplicate/busy windows.
2. Trigger outside then inside PIR for entry, reverse for exit; check both rooms and the corridor total.
3. Vary LDR light levels while occupied; empty rooms must turn lights off.
4. With no emergency active, log in and change the schedule, then verify both doors reject scans outside hours. Use an overnight schedule too. Test emergency evacuation separately using EMERGENCY_SETUP.md.
5. Leave people recorded in rooms and use the admin reboot command. Verify the same users, schedule and counts are restored and lights are recomputed.
6. Verify SD files and serial log queries. Temporarily remove/fail the SD card and check the serial backup and status counter.
7. Check wrong-PIN lockout, session expiry, registration and refusal to delete an occupant.

## Recovery and practical limits

NVS must be retained across resets for recovery. Erasing flash or starting a simulator with a new flash image removes the checkpoint. The first run imports legacy SD users/schedule and starts without prior occupancy; subsequent boots use the NVS checkpoint as authoritative. SD user/config files are exports of that state, not a second independent authority.

Normal building occupancy now changes after a card-authorized clear/blocked/clear doorway cycle. Pending intent is saved before opening. Interrupted intents require reconciliation after restart. One beam per designated door cannot identify a person, determine travel direction, or detect two people sharing one continuous beam break. Room PIR counting also assumes distinguishable single-person sequences. Ambiguity or storage failure is flagged instead of guessing.

If NVS is unreadable, inspect the error and saved data before deciding whether to erase it. The application deliberately does not auto-erase. If SD writes fail, serial backup is observable but is durable only if captured elsewhere. NVS checkpoints are written per state change for this small pilot; sustained large-site traffic would need a wear/performance review.

Normal after-hours restrictions have an explicit emergency exception. This is coursework simulation code; real-building fire detection, emergency egress and power-loss behavior require approved alarm interfaces and suitable exit hardware.

## Ten-minute closing warning and closing-time alarm
While the schedule is open, the first check in the final 10 minutes before closing triggers a 10-second buzzer warning (500 ms on, 500 ms off). All four LCDs show BUILDING CLOSING / IN 10 MINUTES, then update the remaining minutes until closing. Temporary access messages and the entrance admin menu remain usable; the countdown returns after temporary messages expire. Serial and SD logs record the remaining minutes and occupancy. It warns even when recorded occupancy is zero.

The warning is nonblocking and does not change door permissions or occupancy. Both normal entry and exit remain restricted after closing time. Emergency alarm output takes priority and cancels an ongoing closing beep. One warning is issued per closing occurrence during a run, including overnight schedules. A restart in the warning window warns again. Starting or moving the clock into the final ten minutes warns immediately; skipping the entire window cannot issue an advance warning. A changed closing time gets its own warning if it lies in the current warning window.

Configure CLOSING_WARNING_MIN, CLOSING_BUZZ_MS and CLOSING_BEEP_MS in main/main.c. No extra hardware is required.

When the schedule changes from open to closed, a second 10-second pulsing alarm
sounds, all four LCDs show BUILDING CLOSED until reopening, and serial
and SD logs record CLOSING ALARM. It sounds once for that transition, including
overnight schedules. An admin time/schedule change that closes the building also
triggers it. Starting while already closed does not trigger it. Emergency alarm
output takes priority over both closing alarms.

For a quick demo (with the emergency reset), enter `login 2026`,
`time set 2026-10-30 20:49:00`, and `schedule set 05:00 21:00`.
Then enter `time set 2026-10-30 20:50:00` for the advance warning. After its ten
seconds finish, enter `time set 2026-10-30 20:59:55` and wait five seconds for the
closing-time alarm. Restore the correct clock and schedule after testing.

## Main exit requires a room exit first
Normal main-door exit requires a recorded corridor occupant. To leave a room,
trigger its INNER PIR and then OUTER PIR within six seconds; the room count
decreases while building occupancy stays unchanged. Once the sensors clear,
scan the same card at the main exit and press/release the EXIT PASSAGE button.
That verified passage reduces building occupancy only. A scan made while all
recorded occupants are still in rooms is rejected with Leave room first. Exit
reservations prevent room entry from consuming a corridor occupant who is
already passing through the main exit. Emergency evacuation still holds the exit
open without requiring cards or room PIR motion.

PIRs count anonymous movements; they do not identify cardholders. With multiple
occupants, this rule guarantees available corridor capacity, not personal room
identity. Room counts are never automatically decremented by a main-door scan.

User-facing storage/state failure messages now say SYSTEM DOWN (Contact admin on idle door screens). Detailed checkpoint and journal errors remain in diagnostic logs.

Card rejection labels: Card not registered, Card already used, and No entrance recorded. Each uses two LCD rows to fit all words without truncation; the rejection beep and full log reason remain. Card already used means the card currently has a confirmed entry without a confirmed exit, not that it can only be used once.

Display simplification: after-hours scans show ACCESS DENIED / Building closed, and closed idle door screens show BUILDING CLOSED without opening hours. Removed No one inside (the per-card No entrance recorded check covers this), Door busy, Sensor not clear and Counts unverified explanations from access LCDs. Busy/blocked passage checks still refuse authorization with a generic ACCESS DENIED display; uncertain occupancy uses ACCESS DENIED / Contact admin. Detailed diagnostic and checklist messages remain.

## Occupancy labels
The `occupancy` and `status` commands show Room 1, Room 2 and Building count.
Building count includes everyone recorded inside, including people outside the
two monitored rooms. It does not identify a corridor, toilet or other location.
