# Fix loop result: opening widths

The declaration (`docs/FIX_DECLARATION.md`) was committed before any fix code (tag `fixloop-before`). The fix is tagged `fixloop-after`. To regenerate both runs from those tags and rebuild the diff, run `scripts/fixloop.sh`. The readable diff is `docs/fixloop.diff`.

## Outcome against the prediction

| Metric | Before | Predicted | After | Verdict |
|---|---|---|---|---|
| LiDAR opening gate (≤ 2 cm, misses and phantoms count) | 0% (0/14) | ≥ 85% | **93% (13/14) PASS** | met |
| LiDAR median abs width error | 3.5 cm | ≤ 1.0 cm | **0.3 cm** | met (better than predicted) |
| LiDAR mean per-edge error | −1.7 cm | within ±0.5 cm | **+0.3 cm** (full capture), +0.0 and +0.3 (repeats) | met |
| Video opening gate | 0% | still fail | 13% (2/10 GT, 5 phantoms), FAIL | met |
| Photo opening gate | 0% | still fail | 0%, FAIL | met |
| Other gates | — | unchanged | unchanged except **LiDAR calibration: FAIL → PASS** (43/55 → 51/55 covered) | wrong in detail, explained below |

The one prediction that was wrong: LiDAR calibration coverage pools every LiDAR measurement, including opening widths. While the widths were biased by −3.5 cm against a sigma of about 1.3 cm, most of them fell outside their own intervals. Removing the bias brought them back inside. I should have predicted this. It also shows that the earlier calibration failure was this same bug, not mis-sized intervals.

## How the fix got there (two iterations, both in the history)

1. **Decision stump between wall hits and through-ray crossings.** This moved the per-edge mean from −1.7 to −1.25 cm and the gate stayed at 2/14. By the falsification test written in the declaration ("if the per-edge mean is still worse than −1 cm, the hypothesis is incomplete"), this did not count as a fix. Instrumenting the edges (`scripts/diag_jambs.py`) showed the reason. Through-ray crossings stop about 1 cm short of the true edge, because rays that graze the jamb hit the reveal before they reach the wall plane. Reveal hits, blurred by noise along the ray, spill about 1 cm into the opening. The stump could only find the middle of a gap that was itself shifted inward.
2. **The jamb reveal as primary evidence.** Points whose normal runs along the wall and faces into the opening lie *on* the jamb surface, so their median along-wall position is the edge. This is also what a tape measure reads when it measures "between the jambs". The stump remains only as a fallback for openings where no reveal is visible. This version landed at +0.3 cm.

So the declared root cause was right, but incomplete. The cell vote does lose edge cells. The deeper problem was that through-ray evidence cannot localise an edge it is shadowed from.

## What is still wrong

- **One LiDAR miss:** the living-room 1.40 m window comes out 2.6 cm narrow. Its far jamb is seen only at grazing angles from the capture path, so there are fewer than 8 reveal points and the stump fallback is used.
- **Real-world risk:** door casings protrude 1–2 cm proud of the wall. On a real door the reveal median measures *jamb to jamb*, which is the conventional door width, but the casing face could sit in the hit band. This has to be confirmed on the real benchmark (`benchmark/real/`) before the synthetic number is quoted for real use.
- **Video and photo openings** are limited by wall placement and mono-depth error, not by edge localisation. They are the next fix-loop candidate (see the technical report).
