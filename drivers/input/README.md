# Vendored Azoteq TPS43 trackpad driver

Source: https://github.com/beekeeb/zmk_driver_azoteq

- Upstream commit: `c329f30` ("allow user to customize tap-time,tap-distance,hold-time")
- License: MIT (see `LICENSE`), Copyright (c) 2025 Mariano Uvalle
- Copies: `tps43.c`, `tps43.h`, `tps43_idle_sleeper.c` and the devicetree
  bindings in `dts/bindings/input/azoteq,tps43-*.yaml` are unmodified except
  for the polling patch below.

## Local patch: RDY-less polling

The dabase_v2 PCB does not route the TPS43 RDY line, and the upstream driver
only reads the report window from the RDY GPIO interrupt. Without a patch it
would handshake, configure the pad and then never read touch data.

When the devicetree node has no `rdy-gpios`, the driver now reads on a
`k_work_delayable` timer instead:

- `CONFIG_INPUT_TPS43_POLL_INTERVAL_MS` (default 5 ms) while a finger is down
- `CONFIG_INPUT_TPS43_POLL_IDLE_INTERVAL_MS` (default 20 ms) while untouched
- the poll is cancelled when the pad is suspended and restarted on resume
- `EVENT_MODE` stays off without RDY: it suppresses the communication window
  until an event occurs, so every poll would be NACKed
- a NACKed read is retried after 250 us (datasheet 8.8.2, "Force
  Communication": a low-power pad NACKs the first addressing on purpose)
- an empty window between two report windows does not end a touch, and
  `BTN_TOUCH` is not reported by default (`CONFIG_INPUT_TPS43_REPORT_TOUCH`),
  because ZMK maps it to a held left mouse button
- scroll reports both wheel axes by default
  (`CONFIG_INPUT_TPS43_SCROLL_BOTH_AXES`) and carries sub-count remainders
  so slow scrolls are not truncated. NOTE: the IQS5xx gesture engine
  resolves each scroll window to a single axis before reporting (measured:
  dx=0/dy=N or dx=N/dy=0), so diagonal scrolling cannot be produced from
  the scroll gesture on this pad.
- `CONFIG_INPUT_TPS43_POWER_POLICY` (dabase_v2) gates the pad's VCC rail on
  ZMK's activity state, ported from the TrackPoint field fix: cut on idle,
  restore on activity or USB, then re-run the whole init because the IQS5xx
  loses its registers on power loss; the I2C pins go to their sleep state
  (bias disabled) while the rail is down
- `CONFIG_INPUT_TPS43_RAW_DIAG` prints the raw report window for bring-up

Everything else (init sequence, ACK_RESET, ATI, configuration writes,
gestures, power management) is upstream behaviour.
