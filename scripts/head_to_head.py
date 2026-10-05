"""Head-to-head table: our LiDAR-tier errors vs a consumer app's, dimension by dimension.

    python scripts/head_to_head.py reports/real/results.json benchmark/real/head_to_head/consumer.csv \
        -o reports/real/HEAD_TO_HEAD.md

consumer.csv (one row per dimension the consumer app exported, read off its PDF/JSON export):

    capture,dimension,consumer_m
    real_lidar_flat,bedroom/wall/0,3.212
    real_lidar_flat,bedroom/ceiling_height,2.571
    real_lidar_flat,bedroom/opening/door/0.812,0.80

`dimension` uses the GT ids the benchmark emits: <room>/wall/<gt edge index>, <room>/ceiling_height,
<room>/floor_area, <room>/opening/<type>/<gt width>. A tie is |our err| - |their err| <= 5 mm.
Rule from the brief: beat or tie on >= 70% of shared dimensions.
"""
import argparse
import csv
import json
from pathlib import Path

TIE = 0.005


def our_dims(res):
    out = {}
    for s in res["scores"]:
        for r in s["records"]:
            if r["kind"] == "wall_length":
                key = f"{r['room']}/wall/{r['gt_wall'].split('/')[-1]}"
            elif r["kind"] in ("ceiling_height", "floor_area"):
                key = f"{r['room']}/{r['kind']}"
            elif r["kind"] == "opening_width":
                key = f"{r['room']}/opening/{r['type_gt']}/{r['gt']:.3f}"
            else:
                continue
            out[(s["capture"], key)] = r
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("results")
    ap.add_argument("consumer_csv")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--app", default="consumer app (name + version in benchmark/real/head_to_head/APP.txt)")
    a = ap.parse_args()
    ours = our_dims(json.loads(Path(a.results).read_text()))
    rows = []
    with open(a.consumer_csv) as f:
        for row in csv.DictReader(f):
            k = (row["capture"], row["dimension"])
            if k not in ours:
                print(f"skip {k}: not in our results")
                continue
            r = ours[k]
            e_o, e_t = abs(r["err"]), abs(float(row["consumer_m"]) - r["gt"])
            rows.append((k[0], k[1], r["gt"], r["pred"], float(row["consumer_m"]), e_o, e_t,
                         "win" if e_o < e_t - TIE else "tie" if abs(e_o - e_t) <= TIE else "loss"))
    wins = sum(x[-1] in ("win", "tie") for x in rows)
    L = [f"# Head-to-head: propscan (LiDAR tier) vs {a.app}\n",
         "| Capture | Dimension | Tape/laser | Ours | Theirs | Our error | Their error | Result |", "|---|---|---|---|---|---|---|---|"]
    for c, d, gt, po, pt, eo, et, res in rows:
        L.append(f"| {c} | {d} | {gt:.3f} | {po:.3f} | {pt:.3f} | {100 * eo:.1f} cm | {100 * et:.1f} cm | {res} |")
    share = wins / len(rows) if rows else 0.0
    L.append(f"\nBeat or tie on **{wins}/{len(rows)} = {share:.0%}** of shared dimensions "
             f"(gate >= 70%: **{'PASS' if share >= 0.7 else 'FAIL'}**). Tie tolerance 5 mm.")
    text = "\n".join(L) + "\n"
    if a.out:
        Path(a.out).write_text(text)
    print(text)


if __name__ == "__main__":
    main()
