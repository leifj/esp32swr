# esp32swr calibration strategy

Four layers, in order. The firmware already handles layer 0 automatically.

## 0. Zero / offset (automatic)

`calibrateOffsets()` measures both channels with no RF at boot and
subtracts the baseline before conversion. Verify occasionally: with the
rig idle, `/api/status` should show `valid:false`, `fwd 0.000`,
`rev 0.000`. If boot-time calibration logs "RF detected at boot", reboot
with the rig idle.

## 1. Forward scale — one trusted reference, mid-range

Reference options (best first):

- Scope across the 50 ohm load: `P = Vpp^2 / 400` (3% accurate scope is
  the most trustworthy reference in the shack)
- The HL2's calibrated power meter (quisk `power_meter_std_calibrations`)

Key a steady carrier (the quisk ATU tune carrier is ideal: constant,
unmodulated). Use ~30-50% of normal operating power, e.g. 5-10 W.

    CAL_FWD_new = CAL_FWD_old * sqrt(P_ref / P_fwd_measured)

## 2. Detector law — multiple points

Repeat at 1, 2, 5, 10, 25 W (raise `fwd_max` temporarily near 50 W).
Plot `P_meter / P_ref` vs level. Diode detectors are square-law only at
low levels and bend toward linear near the top; a single `W = k*V^2`
constant can drift 10-20%.

- Within tolerance: stop here.
- Otherwise: replace the constant with a piecewise interpolation table
  over ADC code `{code -> watts}` (same model quisk uses for the HL2).
  Biggest accuracy win at both ends of the range.

## 3. Directivity correction — what makes low SWR meaningful

A coupler leaks forward power into the reverse port:

    P_rev_measured = P_rev_true + D * P_fwd

With a good 50 ohm dummy load, apply a known forward level and record
`D = rev_measured / fwd_measured` at several frequencies (3.5 / 7 / 14 /
28 MHz). Good bridges: -30 to -50 dB. Correct before SWR:

    P_rev = max(P_rev_measured - D * P_fwd, 0)

Without this, a perfect load reads ~1.008 and a real 1.05 match is
indistinguishable from leakage.

## 4. Validate with known mismatches

Low power (1-2 W so the protection relay stays open; or disable
Auto-relock):

| Load                        | SWR |
|-----------------------------|-----|
| 2x 50 ohm parallel (25)     | 2.0 |
| 3x 50 ohm parallel (16.7)   | 3.0 |
| 2x 50 ohm series (100)      | 2.0 |
| open / short                | inf (99 sentinel) |

This validates the reverse scale relative to forward. If forward is
right, `CAL_REV` should equal `CAL_FWD` for symmetric couplers; only the
directivity term is usually needed.

## Frequency / band trimming (optional, last)

Coupler coupling varies 1-2 dB across HF. If stage 4 shows a systematic
per-band error, add a per-band table selected by an API call that quisk
sends on ChangeBand (same pattern as the ATU-100 band memory).

## Practical notes

- Calibrate warm, with the enclosure closed (diode drift)
- Average 5-10 readings per point
- Same cable/connectors throughout (adapters cost 0.1-0.3 dB)
- Record final `CAL_FWD` / `CAL_REV` / `D` values in the commit message
