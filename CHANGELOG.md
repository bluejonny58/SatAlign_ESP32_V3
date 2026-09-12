# Changelog

## V3.1.5 - current

- Manual fine-adjust Web UI reorganized for direct feedback while clicking.
- The manual fine-alignment controls now label the live values as user-facing `Signalstärke` and `Feinwert` instead of `RF` and `OPT`.
- A new fine-adjustment reference is set automatically when opening `Manuell`: AZ starts at `0` steps; EL stores the current MPU angle.
- Position display now shows the reference explicitly: AZ as `0 -> +/-n steps`; EL as `reference angle -> current angle (delta)`.
- Reset controls and pulse/PWM values were removed from the user-facing fine-adjustment rows; pulse settings remain centrally configurable in `settings.cpp`.
- The separate lower signal/diagnostic card was removed from the manual page; Signalstärke and Feinwert remain directly at the fine-step controls.
- Technical status is compacted to three rows: AZ/EL motor state, Hall sensors, and EL angle with soft limits; duplicate Web/Live state rows were removed.
- Search Web UI presents the actual RF percentage and quality label instead of voltage/ADC as the primary user-facing signal value; raw values remain available in diagnostic contexts.
- User-facing RF quality classes are derived from the common percentage scale: `<80 % = weak`, `80-<85 % = usable`, `85-<95 % = good`, `>=95 % = very good`. This classification is display-only; the AUTO candidate threshold remains 80 %.
- Main `SatAlign_ESP32_V3.ino` header now explicitly identifies firmware V3.1.5.
- Firmware version raised to 3.1.5. AUTO search logic, 80 % candidate threshold, RF normalization and tested motor-direction mapping are unchanged.

## V3.1.4 - stable

- RF filter changed from 0.75 to 0.50 after a reproducible RF/azimuth comparison test.
- Normal AUTO candidate minimum remains 80 %.
- RF display cap adjusted from 95 % to 100 %; AUTO candidate minimum remains unchanged.
- TFT, Web UI, serial diagnosis and AUTO search use the same RF percentage evaluation.
- Compact AUTO serial logging added and cleaned up for field diagnostics.
- Redundant periodic AUTO output suppressed while important events remain visible.
- Stable AUTO search logic retained; no experimental peak-return logic included.
- GeniusBulli uses fixed IP 192.168.4.15.
- ArduinoOTA and ElegantOTA are available in parallel.
- mDNS is not used.
- Legacy automatic "Signal optimieren" workflow removed from the active user flow.
- Existing `tools/SatAlign_ESP32_V3_InstallTest/` hardware diagnostic sketch retained and documented explicitly.
- Documentation aligned with current center/elevation behavior: PLUS/MINUS use short 250 ms pulses; obsolete 10-second boot-adjust window removed from documentation.

## V3.1.3

- Normal AUTO candidate threshold raised to 80 % based on field testing.

## V3.1.2

- RF strong reference adjusted to 700 ADC.
- Usable RF percentage display capped at 95 %.

## V3.1.1

- ElegantOTA added alongside ArduinoOTA.
- RF percentage calculation unified between display and AUTO search.
