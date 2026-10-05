"""Streamlit UI to upload zip captures and run the propscan pipeline."""
from __future__ import annotations

import json
import shutil
import zipfile
from pathlib import Path

import streamlit as st

ROOT = Path(__file__).resolve().parents[1]
UPLOAD = ROOT / "data" / "uploads"
OUT = ROOT / "out"


def _extract_zip(zip_path: Path) -> Path:
    dest = UPLOAD / zip_path.stem
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)
    with zipfile.ZipFile(zip_path) as zf:
        zf.extractall(dest, members=[m for m in zf.namelist() if not m.startswith("__MACOSX")])
    kids = [p for p in dest.iterdir() if not p.name.startswith(".")]
    if len(kids) == 1 and kids[0].is_dir():
        return kids[0]
    return dest


def _resolve_upload(name: str, raw: bytes) -> Path:
    UPLOAD.mkdir(parents=True, exist_ok=True)
    path = UPLOAD / name
    path.write_bytes(raw)
    if path.suffix.lower() == ".zip":
        return _extract_zip(path)
    return path


def _summary(plan: dict) -> None:
    cap = plan.get("capture", {})
    st.subheader("Summary")
    st.write(
        f"**Tier:** {cap.get('tier')} · **Rooms:** {len(plan.get('rooms', []))} · "
        f"**Footprint:** {plan['stitched_plan']['footprint_area']['value']:.2f} m²"
    )
    if plan.get("warnings"):
        for w in plan["warnings"][:12]:
            st.warning(w)
        if len(plan["warnings"]) > 12:
            st.caption(f"+ {len(plan['warnings']) - 12} more warnings in plan.json")


def main() -> None:
    st.set_page_config(page_title="Property Scan", layout="wide")
    st.title("Handheld property scan")
    st.caption("Upload interviewer zips → run pipeline → view plan and JSON. No data leaves this machine.")

    with st.sidebar:
        st.header("Run options")
        depth = st.selectbox("Depth", ["auto", "model"], help="model = live Depth Anything V2 (needs make models)")
        drift = st.checkbox("Drift correction", value=True)
        run_id = st.text_input("Output folder name", value="latest")

    uploaded = st.file_uploader(
        "Capture zip or file",
        type=["zip", "mov", "mp4", "m4v"],
        accept_multiple_files=True,
        help="Add up to three zips (lidar, video, photo) or run one at a time.",
    )

    if not uploaded:
        st.info("Place files under `data/uploads/` or upload here. See `data/README.md`.")
        return

    picks = st.multiselect("Files to process", [f.name for f in uploaded], default=[uploaded[0].name])
    if st.button("Run pipeline", type="primary"):
        from propscan.config import load_config
        from propscan.ingest.detect import detect_tier
        from propscan.pipeline import run_capture

        cfg = load_config(overrides={"depth_model": {"source": depth}})
        by_name = {f.name: f for f in uploaded}
        for name in picks:
            capture_path = _resolve_upload(name, by_name[name].getvalue())
            tier, root = detect_tier(str(capture_path))
            out_dir = OUT / f"{run_id}_{tier}" if len(picks) > 1 else OUT / run_id
            with st.spinner(f"Running {name} ({tier})…"):
                plan = run_capture(
                    str(root if root.is_dir() else capture_path),
                    str(out_dir),
                    cfg,
                    tier=tier,
                    drift=drift,
                    capture_id=Path(name).stem,
                    verbose=False,
                )
            st.success(f"Wrote {out_dir}")
            _summary(plan)
            png = out_dir / "plan.png"
            if png.exists():
                st.image(str(png), caption="Floor plan")
            with st.expander("plan.json"):
                st.json(plan)
            st.download_button(
                f"Download {tier} plan.json",
                json.dumps(plan, indent=2),
                file_name=f"plan_{tier}.json",
                mime="application/json",
            )


if __name__ == "__main__":
    main()
