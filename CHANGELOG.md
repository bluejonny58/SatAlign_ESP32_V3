# Changelog

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

## V3.1.3

- Normal AUTO candidate threshold raised to 80 % based on field testing.

## V3.1.2

- RF strong reference adjusted to 700 ADC.
- Usable RF percentage display capped at 95 %.

## V3.1.1

- ElegantOTA added alongside ArduinoOTA.
- RF percentage calculation unified between display and AUTO search.
