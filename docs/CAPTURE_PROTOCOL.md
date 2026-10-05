# Capture protocol (one page, follow literally)

**Route 2: stock apps only.** Pick **one** tier per capture. Every tier produces the same plan; more sensor data gives tighter numbers.

| Tier | App | Phone |
|---|---|---|
| **LiDAR** | **Stray Scanner** (App Store, free, by Kenneth Blomqvist) | iPhone 12 Pro or newer **Pro** (has LiDAR) |
| **Video** | Built-in **Camera** app, *Video* mode | Any iPhone 15 or newer |
| **Photos** | Built-in **Camera** app, *Photo* mode, **0.5×** lens | Any iPhone 15 or newer |

## Before you start (once, 2 minutes)
1. *Settings → Camera → Formats →* **Most Compatible** (makes JPEG/H.264; HEIC also works but is slower).
2. *Settings → Camera → Record Video →* **1080p HD at 30 fps**. Turn **Action Mode off**. Do **not** zoom during video.
3. Turn on every light in every room and open the curtains. Open all interior doors fully.
4. For LiDAR: install Stray Scanner. Leave its settings at default.

## How to walk (all tiers)
- Hold the phone **upright at chest height**. Move **slowly**: about one step per second, and turn no faster than a slow head turn.
- In **each room**, stand near the middle and **turn a full circle**. Tilt up so the **ceiling-wall edge** is visible all the way round, then tilt down so the **floor-wall edge** is visible.
- Walk **through every doorway**. Pause 1 second in each doorway, facing into the next room.
- **Finish where you started** (return to the first room and look at the same wall again). This lets the software close the loop.
- **Avoid:** pointing at a mirror or a window for more than a second; your own body or fingers in the frame; people walking through.
- **Time:** about 45 seconds per room. A 4-room flat takes about 3–4 minutes. If a capture runs over 10 minutes, split it into two.

## Tier specifics
**LiDAR (Stray Scanner):** press record, walk the whole property once as above, then press stop. Do not pause the recording.
**Video:** same walk, as one continuous clip in landscape **or** portrait (not both).
**Photos:** **2 to 8 photos per room** with the **0.5× lens** (tap "0.5" above the shutter), in **landscape**. Stand in the middle and take one photo, turn about 45° (an eighth of a turn), take the next, and repeat. **Every doorway must appear in at least one photo of each room it joins.** Consecutive photos must overlap by about half. Small rooms: 4 photos from the middle at 90° steps. Make **one album or folder per room**, named after the room (e.g. `kitchen`).

## Handing the files over
| Tier | What to copy to the laptop | How |
|---|---|---|
| LiDAR | The scan folder (contains `rgb.mp4`, `depth/`, `odometry.csv`, `camera_matrix.csv`) | Stray Scanner → share/export → AirDrop to the Mac, **or** Files app → *On My iPhone → Stray Scanner* |
| Video | The `.MOV` file | AirDrop (choose *Options → All Photos Data* so it is not re-encoded) |
| Photos | One folder per room, with the original photos inside | Select the room's photos → AirDrop (*Options → All Photos Data*) → drop them into `my_flat/<room name>/` |

Optional (photos only): a text file `my_flat/connections.txt` with one line per door, such as `hall - kitchen`. This helps if two doors look identical.

## Run
```
propscan run <the folder or file you copied> -o out/my_flat
```
The plan is `out/my_flat/plan.png`, and the data is `out/my_flat/plan.json`.

## Hard surfaces
**Mirrors** are detected and their phantom "room" is removed, but do not stand facing one. **Glass** returns no LiDAR depth, so windows are measured from their frames. **Wet or glossy floors** and **low light** are reported as warnings and the intervals widen. Turn on the lights rather than relying on that.
