# grblhal-scanline

grblHAL plugin. Adds a raster scanline move: one planner block per line, with laser power set per pixel from the step interrupt.

## Commands

| Command | Action |
|---|---|
| `$SLH=<hex>` | Append pixel data for the next `M390`. Repeat to send a line in chunks. |
| `M390 X.. Y.. Z.. [I.. J.. K..] P.. Q.. [R..] [S..] [D..] [E1]` | Scan to the target, spreading the pixels evenly along the move. |

### M390 words

| Word | Meaning |
|---|---|
| `X` `Y` `Z` | Target: the end of the last pixel. Follows G90/G91, G20/G21 and work offsets. At least one is required. |
| `I` `J` `K` | Distance from the first pixel to the target along X, Y, Z. Optional; without them the pixels start at the current position. An axis left out has distance 0. |
| `P` | Pixel count. |
| `Q` | Bits per pixel: 1, 2, 4 or 8. |
| `R` | Overscan distance, at least one step. Optional; default is the distance needed to reach `F` from rest plus the distance covered at `F` in one acceleration tick. |
| `E` | `E1`: the line continues in the next `M390`. Optional. |
| `S` | Power of the highest pixel value. Optional; default is the modal `S`. |
| `D` | Power of pixel value 0, in `S` units, at most `S`. Optional; default is the last `D`. |

`F` and `S` are modal, as on a `G1` line. `D` is modal across `M390` lines; reset, `M2` and `M30` set it to 0.

### Pixel data

- Packed MSB first. The first pixel is at the start given by `I` `J` `K`, or at the current position without them.
- Length is `ceil(P * Q / 8)` bytes.
- Hex, two characters per byte, either case.

### Power

- Pixel value `v` gives `D + (S - D) * v / (2^Q - 1)`. With `D` at 0, value 0 is off.
- A change of pixel value takes effect on the step that reaches the pixel, as a change of `S` between `G1` moves does.
- `M3`: constant power. `M4`: power scaled with speed.
- `M5`: the move runs with the laser off.
- The laser is off outside the pixels and when the move ends.

### Overscan

The tool rapids from wherever it is to the overscan distance before the start, scans to the same distance beyond the target and stops there. Every `M390` has overscan, so a pixel never starts on the first step of a move.

### Continued lines

A line longer than the pixel pool is sent as consecutive `M390` moves, all but the last with `E1`.

- `E1` leaves out the overscan after the target.
- An `M390` that starts where an `E1` move ended, in the same direction, has no overscan before the start. Its first pixel starts on the last step of the move before it.
- A different direction from that position is rejected.

## Example

```
M4 S1000
$SLH=F0F0
M390 X26 Y20 I16 P16 Q1 F30000
$SLH=0F0F
M390 X10 Y20.1 I-16 P16 Q1
```

## Errors

| Status | Cause |
|---|---|
| 2 | `$SLH` data is not valid hex |
| 4 | `S` or `D` is negative |
| 11 | `$SLH` data exceeds the pixel pool |
| 20 | Laser mode is not active |
| 22 | No feed rate |
| 23 | `P` or `Q` is not an integer |
| 26 | No axis word |
| 28 | `P` or `Q` is missing |
| 39 | Value out of range, zero-length move, more pixels than steps, `R` shorter than one step, `D` above `S`, or data length does not match `P` and `Q` |

A rejected `M390` discards the pending pixel data. A `$SLH` that follows a failed `$SLH` starts the data again.

## Discovery

`$I` reports `[SCANLINE:1,<pixel pool bytes>]`. One line can use the whole pool.

## Limits

- Requires laser mode (`$32=1`).
- Pixel count cannot exceed the step count of the longest axis in the move.
- `M390` does not move in check mode.
- Automatic overscan uses the programmed feed rate and the acceleration and maximum rate settings of the axes in the move. A feed override above 100 % in effect when the line is queued is included; lines already queued when it is raised are not adjusted.

## Build

| Define | Default | Meaning |
|---|---|---|
| `SCANLINE_ENABLE` | unset | Set to 1 to compile the plugin |
| `SCANLINE_POOL_SIZE` | 32768 | Pixel data bytes shared by all buffered lines |
| `SCANLINE_QUEUE_SIZE` | 8 | Lines buffered ahead of motion |
| `SCANLINE_PERIOD_DELAY` | 1 on STM32 drivers, else 0 | Ticks before the driver's step timer applies a new period |

Call `scanline_init()` from `my_plugin_init()`.

## License

GPLv3.
