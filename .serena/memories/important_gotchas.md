# Important Gotchas and Warnings

## 1. Always Use `uv run` Prefix

All Python tools must be run via `uv run`:
```bash
# ✅ Correct
uv run west build .
uv run pytest

# ❌ Wrong (won't use project dependencies)
west build .
pytest
```

## 2. West Context

West commands must be run from the repository root or within `benji-rt`. West uses `.west/config` to understand workspace structure.

## 3. Yocto Build Times

- First build of `meta-benji` takes **hours** and uses **tens of GB** of disk space
- Moon caching is disabled for Yocto tasks (BitBake handles caching)
- Don't interrupt builds - use `moon run meta-benji:clean` if needed

## 4. Secrets Management

- `meta-benji/secrets.yaml.tmpl` is a template
- Copy to `secrets.yaml` and fill in actual secrets
- **NEVER commit `secrets.yaml`** (it's in .gitignore)

## 5. Jupyter Notebooks in Git

- Notebooks in `doc/gnc/notebooks/` are tracked with execution output
- `doc:gen-notebooks` task re-executes them before building docs
- This ensures documentation reflects current code

## 6. Moon Task Dependencies

Tasks automatically run dependencies:
- `benji-rt:flash` depends on `benji-rt:build`
- `doc:build` depends on `doc:gen-notebooks`
- `meta-benji:build` depends on `meta-benji:setup`

## 7. Zephyr Configuration

- `prj.conf` contains Kconfig options
- Board-specific settings in `boards/*.overlay`
- Current target: `adafruit_feather_stm32f405`
- C++23 is enabled

## 8. Python Version

Project requires **Python 3.13+** (from `pyproject.toml`).

## 9. Documentation Build Order

Build has specific order due to dependencies:
1. Execute Jupyter notebooks (`gen-notebooks`)
2. Build Sphinx documentation (`build`)

Use `moon run doc:build` which handles this automatically.

## 10. CI/CD Triggers

- Documentation builds and deploys on push to `main` branch
- GitHub Pages serves from `doc/_build/html`

## 11. Pre-commit Hooks Required

Run `just bootstrap` to install pre-commit hooks, otherwise commits may fail validation.
