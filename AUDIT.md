# Project audit - 2026-10-31

The audited working tree builds and passes the checks listed below. Changes are
not automatically committed or pushed.

## Corrected issues

- Removed committed merge-conflict markers from `validation.json` and restored
  valid JSON. Git had no unmerged index entries; the markers were saved as text.
- Fixed PIR sequence handling: a second sensor arriving after six seconds is now
  rejected before occupancy changes. Added late-trigger and exact-boundary checks.
- Updated the native runner to preserve separately recorded build and simulator
  evidence, and record a source hash rather than overwriting those results.
- Replaced obsolete README references to missing modules/setup documents with the
  current five-source build, operating sequence and supported commands.
- Replaced an invalid LED colour and removed redundant switch attributes. Corrected
  the emergency-wiring helper's power-pin name to match the supplied S3 board.
- Corrected live lamp tests: open relay contacts can float, so OFF is verified by
  the driven control pin and visible lamp output, not an assumed open-contact 0.
  Allow relay settling before reading its state.

## Checks passed

- No remaining conflict markers or unmerged Git index entries.
- All project JSON/TOML/YAML and Python syntax; unique diagram IDs, valid endpoint
  references, expected device wiring and current firmware paths.
- All five application C files checked by native and target compilers.
- 315 production-logic assertions; 400 concurrent room transitions.
- AddressSanitizer and UndefinedBehaviorSanitizer enabled and passed.
- Complete ESP-IDF v6.0.2 ESP32-S3 build and app-partition size check.
- Six live Wokwi scenarios: Blue and Green taps; ordered room-exit sequence and
  main-exit refusal; both lamps; smoke-trigger and reset guard; closing displays,
  both closing alarms, emergency button and administrator reset.
- Actual lamp ON/OFF screenshots for both rooms and LCD screenshots visually
  inspected. GPIO18 waveform measured at approximately 2000 Hz during the live
  closing/emergency scenario.

Machine-readable results and source/firmware hashes are in `validation.json`.
Live transcripts, scenario results and waveform are under `tests/wokwi/audit`.
The standard scenarios are under `tests/wokwi`; README lists their commands.

## Scope and remaining practical limits

The tests establish the checked software/simulator behavior, not a guarantee of
perfection or a physical hardware certification. PIRs provide anonymous movement
counts, so they cannot identify a particular person's room. The room-sequence
automation drives the production GPIO inputs through a dedicated test fixture;
the normal PIR user-interface motion controls are not automated. Lighting tests
use the room-entry test hook to isolate the LDR and actual lamp circuit.

Buzzer signal generation is verified, but audible playback through the user's
VS Code and speakers still depends on the local audio output. ESP-IDF emits
nonfatal SDK Kconfig notifications; these did not prevent compilation or execution.

Only the current local tree was checked for merge conflicts. Future changes from
other branches or the remote can introduce new conflicts.
