PY ?= .venv/bin/python
PROPSCAN ?= .venv/bin/propscan
MANIFEST ?= data/synthetic/apartment_a/manifest.yaml

.PHONY: install models ui synth bench bench-model calibrate fixloop test real clean

install:            ## venv + package + ML + UI extras (photo/video tiers)
	python3 -m venv .venv
	$(PY) -m pip install -q --upgrade pip
	$(PY) -m pip install -q -e ".[ml,ui,dev]"

models:             ## depth model weights -> models/ (~100 MB)
	scripts/fetch_models.sh

ui:                 ## Streamlit test UI (upload zips, view plan)
	scripts/run_ui.sh

synth:              ## synthetic benchmark captures (deterministic)
	$(PROPSCAN) synth

bench: synth        ## every gate, all tiers -> reports/synthetic/REPORT.md
	$(PROPSCAN) bench $(MANIFEST) -o reports/synthetic

bench-model: synth  ## photo+video tiers with the live depth model instead of emulated depth
	$(PROPSCAN) bench $(MANIFEST) -o reports/synthetic_modeldepth --only photo,video --depth model

calibrate:          ## fit interval multipliers from benchmark residuals
	$(PROPSCAN) calibrate reports/synthetic/results.json

fixloop: synth      ## regenerate fix-loop before/after from git tags + diff
	scripts/fixloop.sh

real:               ## the real benchmark (needs benchmark/real/manifest.yaml)
	$(PROPSCAN) bench benchmark/real/manifest.yaml -o reports/real

test:
	$(PY) -m pytest -q tests

clean:
	rm -rf out runs .pytest_cache
