# AGENTS.md - Developer Guide for Benji

## Project Overview

**Benji** is a small autonomous security robot designed for indoor surveillance and monitoring. The project implements a dual-processor architecture:

1. **Benji RT** - Real-time control core running Zephyr RTOS on an MCU (STM32F405)
2. **Benji Compute** - Compute platform running embedded Linux (Yocto/AutonomOS) on a Raspberry Pi 5

The project also includes extensive documentation with Jupyter notebooks for Guidance, Navigation, and Control (GNC) algorithms.

### Architecture Context (from ADRs)

- **Split Architecture**: Compute-intensive tasks (image processing, path planning) run on Linux SBC; real-time control (motor control, sensor acquisition) runs on MCU
- **MCU Platform**: Zephyr RTOS chosen for extensive device support, DFU capabilities, and hardware abstraction
- **Compute Platform**: Yocto-based embedded Linux for reproducibility, customization, and portability
- **Versioning**: All applications built and versioned as a single entity

### Hardware Baseline

**Sensors:**
- Camera (image processing)
- 6-axis IMU (accelerometer + gyro)
- 3-axis magnetometer
- Linear ToF distance sensor
- 4x quadrature encoders (wheel odometry)

**Actuators:**
- 4x DC gearmotors with individual drivers
- 1x servo motor (camera tilt)

## Technology Stack

- **Python**: 3.13+ (project language)
- **Package Manager**: `uv` (fast Python package installer and resolver)
- **Build System**: `moon` (monorepo task runner)
- **West**: Zephyr's meta-tool for managing repositories
- **Kas**: Tool for Yocto project setup
- **Just**: Command runner (wrapper around moon tasks)
- **Documentation**: Sphinx with MyST (Markdown), nbsphinx (Jupyter notebooks)
- **Embedded**: Zephyr RTOS, Yocto/BitBake
- **Commit Convention**: Commitizen with conventional commits
- **Pre-commit hooks**: Trailing whitespace, EOF fixer, large files check, commitizen

## Essential Commands

### Project Setup

```bash
# Initial setup - install pre-commit hooks
just bootstrap

# Set up all projects (updates west modules)
just setup
# Equivalent to: moon run :setup
# Which runs: uv run west update

# Install Python dependencies (if needed manually)
uv sync --locked --all-extras --dev
```

### Building

```bash
# Build specific target
just build <target>
# Example: just build benji-rt
# Equivalent to: moon run benji-rt:build

# Build all projects
just build-all
# Equivalent to: moon run :build

# Clean all projects
just clean
# Equivalent to: moon run :clean
```

### Benji RT (Zephyr/Embedded)

```bash
# Build the RT application
moon run benji-rt:build
# Runs: uv run west build benji-rt

# Flash to target device (via J-Link)
moon run benji-rt:flash
# Runs: uv run west flash --runner jlink

# Direct west commands (must use uv run prefix)
uv run west build benji-rt
uv run west flash --runner jlink
```

### Benji Compute (Yocto/Linux)

```bash
# Setup build environment
moon run meta-benji:setup
# Runs: uv run kas-container checkout benji-devel.yaml

# Build the compute platform image
moon run meta-benji:build
# Runs: uv run kas-container build benji-devel.yaml

# Clean build artifacts
moon run meta-benji:clean

# Complete wipe (sources + build + work directories)
moon run meta-benji:spotless
# WARNING: This removes sources/ and build/ directories

# Flash (not yet implemented)
moon run meta-benji:flash
```

**Note**: Yocto builds do not use Moon caching (handled by BitBake). Builds are large and time-consuming.

### Documentation

```bash
# Generate executed Jupyter notebooks
moon run doc:gen-notebooks
# Runs: uv run jupyter execute gnc/notebooks/*.ipynb

# Build documentation
moon run doc:build
# Runs: uv run sphinx-build -M html . _build

# Clean documentation build
moon run doc:clean
# Runs: uv run sphinx-build -M clean . _build

# View built docs
# Open doc/_build/html/index.html in browser
```

### Running Tools Directly

Always prefix commands with `uv run` to use the project's virtual environment:

```bash
uv run west --version
uv run pytest
uv run jupyter notebook
uv run sphinx-build
uv run kas-container --help
```

### Listing Available Tasks

```bash
# List Just recipes
just help
# Or: just --list

# List Moon tasks for a specific project
moon run --help
# Or inspect moon.yml files in project directories
```

## Code Organization

```
benji/
├── benji-rt/              # Zephyr RTOS real-time control application
│   ├── src/               # C/C++ source files
│   ├── inc/               # Header files
│   ├── boards/            # Board-specific overlays
│   ├── CMakeLists.txt     # CMake build configuration
│   ├── prj.conf           # Zephyr project configuration
│   └── moon.yml           # Moon task definitions
├── meta-benji/            # Yocto layer for compute platform
│   ├── benji-devel.yaml   # Kas configuration for development image
│   ├── secrets.yaml.tmpl  # Template for secrets (don't commit secrets.yaml)
│   └── moon.yml           # Moon task definitions
├── doc/                   # Sphinx documentation
│   ├── adr/               # Architectural Decision Records
│   ├── gnc/               # Guidance, Navigation, Control docs
│   │   └── notebooks/     # Jupyter notebooks with algorithms
│   ├── architecture/      # Architecture documentation
│   ├── hardware/          # Hardware documentation
│   ├── projects/          # Project-specific docs
│   ├── conf.py            # Sphinx configuration
│   └── moon.yml           # Moon task definitions
├── deps/                  # West-managed dependencies (Zephyr, etc.)
├── .moon/                 # Moon cache and workspace config
├── pyproject.toml         # Python project dependencies
├── uv.lock                # Locked dependency versions
├── west.yaml              # West manifest
├── Justfile               # Just command recipes
└── moon.yml               # Root Moon task definitions
```

## Project Structure (Moon Monorepo)

The workspace is managed by Moon and defined in `.moon/workspace.yml`:

```yaml
projects:
  - "."           # Root project
  - "benji-rt"    # Zephyr RT application
  - "doc"         # Documentation
  - "meta-benji"  # Yocto compute platform
```

Each project has its own `moon.yml` with task definitions.

## Naming Conventions

### Python Code (GNC Notebooks)

- **Module names**: lowercase with underscores (`simulation.py`, `kalman.py`, `lqr_controller.py`)
- **Function names**: lowercase with underscores (`angle_wrap`, `discretize_system`, `generate_smooth_reference_trajectory`)
- **Class names**: PascalCase (`TankDriveSimulation`, `KalmanFilter`, `SwitchingLQRController`)
- **Constants**: UPPERCASE with underscores
- **Docstrings**: Google style with Args/Returns sections
- **Type hints**: Used where appropriate

### C/C++ Code (Benji RT)

- **Files**: lowercase with underscores (`main.c`)
- **Macros**: UPPERCASE with underscores (`HELLO_INTERVAL_MS`, `CONFIG_LOG`)
- **Functions**: Follows Zephyr conventions (typically lowercase with underscores)
- **Indentation**: Appears to be spaces (verify with existing code)
- **C++ Standard**: C++23 enabled (`CONFIG_STD_CPP23=y`)

### Configuration Files

- **Zephyr configs**: `CONFIG_<SUBSYSTEM>_<OPTION>` format in `prj.conf`
- **Moon tasks**: lowercase with hyphens (`gen-notebooks`, `build-all`)
- **Just recipes**: lowercase with hyphens (mirrors moon tasks)

## Testing Approach

**Note**: Testing infrastructure not yet established in the codebase. When adding tests:

- Python: Use `pytest` (already in dependencies)
- Run tests via: `uv run pytest`
- Consider adding moon tasks for test execution

## Documentation Patterns

### Sphinx Configuration

- **Formats supported**: Markdown (`.md`), reStructuredText (`.rst`)
- **Theme**: sphinx_rtd_theme (Read the Docs)
- **Extensions**:
  - `myst_parser` - Markdown support with MyST extensions
  - `nbsphinx` - Jupyter notebook integration
  - `sphinx.ext.autodoc` - Auto-generate from docstrings
  - `sphinx.ext.graphviz` - Graphviz diagrams
  - `sphinxcontrib.programoutput` - Include command output
  - `sphinxcontrib.jquery` - jQuery support for theme
- **Math**: Dollar math enabled (`$x = y$` inline, `$$...$$` display)

### Jupyter Notebooks

- Located in `doc/gnc/notebooks/`
- Python modules (`.py`) provide shared utilities
- Notebooks are executed before building docs (`doc:gen-notebooks` task)
- Naming: Title Case with spaces (e.g., `Model Predictive Control.ipynb`)

### Markdown Documents

- Use MyST markdown dialect
- Table of contents: ` ```{toctree}` blocks
- Math: `$inline$` or `$$display$$`
- ADRs follow standard format with Status/Context/Decision/Consequences

## Git Workflow

### Commit Messages

Uses **Commitizen** with conventional commits:

```
<type>(<scope>): <subject>

<body>

<footer>
```

**Types**: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`, `build`, `ci`

**Examples from history**:
```
build: use uv to run things instead of relying on the venv
docs(adr): add ADRs discussing the high-level domain split and the OS choices
ci(doc): build and publish docs to github pages
```

### Pre-commit Hooks

Automatically enforced (after `just bootstrap`):

1. **Pre-commit stage**:
   - Trailing whitespace removal
   - End of file fixer
   - Large file check

2. **Commit-msg stage**:
   - Commitizen validation

3. **Pre-push stage**:
   - Commitizen branch validation

### CI/CD

GitHub Actions workflow (`.github/workflows/build-docs.yaml`):

1. Checkout code
2. Setup Moon toolchain
3. Setup Python (from `pyproject.toml`)
4. Install `uv`
5. Install dependencies: `uv sync --locked --all-extras --dev`
6. Build docs: `moon run doc:build`
7. Deploy to GitHub Pages (from `doc/_build/html`)

**Triggered on**: Push to `main` branch

## Important Gotchas

### 1. Always Use `uv run` Prefix

All Python tools must be run via `uv run` to ensure the correct environment:

```bash
# ✅ Correct
uv run west build .
uv run pytest
uv run jupyter notebook

# ❌ Wrong (won't use project dependencies)
west build .
pytest
```

### 2. West Context

West commands must be run from the repository root or within the `benji-rt` directory. West uses `.west/config` to understand the workspace structure.

### 3. Yocto Build Times

- First build of `meta-benji` can take **hours** and use **tens of GB** of disk space
- Moon caching is disabled for Yocto tasks (BitBake handles its own caching)
- Don't interrupt builds - use `moon run meta-benji:clean` if needed

### 4. Secrets Management

- `meta-benji/secrets.yaml.tmpl` is a template
- Copy to `secrets.yaml` and fill in actual secrets
- **Never commit `secrets.yaml`** (it's in `.gitignore`)

### 5. Jupyter Notebooks in Git

- Notebooks in `doc/gnc/notebooks/` are tracked with execution output
- The `doc:gen-notebooks` task re-executes them before building docs
- This ensures documentation always reflects current code

### 6. Moon Task Dependencies

Moon tasks can have dependencies. For example:

- `benji-rt:flash` depends on `benji-rt:build`
- `doc:build` depends on `doc:gen-notebooks`
- `meta-benji:build` depends on `meta-benji:setup`

Running a task automatically runs its dependencies.

### 7. Zephyr Configuration

- `prj.conf` contains Kconfig options
- Board-specific settings go in `boards/*.overlay` (device tree overlays)
- Currently targets: `adafruit_feather_stm32f405`
- C++23 is enabled for the Zephyr application

### 8. Modified Files in Git

Current state (as of conversation start) shows modified files:

```
M .gitignore
M doc/architecture/index.md
M doc/conf.py
M doc/gnc/index.md
M doc/moon.yml
M pyproject.toml
M uv.lock
?? doc/architecture/sw_arch.drawio
?? doc/gnc/notebooks/
```

Be aware of uncommitted changes before making modifications.

### 9. Python Version

Project requires **Python 3.13+** (specified in `pyproject.toml`). Ensure your system has this version or use a version manager.

### 10. Documentation Build Order

The documentation build has a specific order due to dependencies:

1. Execute Jupyter notebooks (`gen-notebooks`)
2. Build Sphinx documentation (`build`)

Always use `moon run doc:build` which handles this automatically.

## GNC Conventions (from Documentation)

### Variables and Symbols

| Symbol | Name | Unit |
|--------|------|------|
| $L$ | Wheelbase | m |
| $\tau$ | Time constant for motor/drivetrain lag | s |
| $v_0$ | Speed about which to linearize | m/s |
| $x, y$ | Position in global frame | m |
| $\theta$ | Heading angle | rad |
| $v_l, v_r$ | Left and right wheel speeds | m/s |

### Notebook Modules

Shared Python modules in `doc/gnc/notebooks/`:

- `simulation.py` - Tank drive simulation, trajectory generation, system discretization
- `kalman.py` - Kalman filter implementation
- `lqr_controller.py` - LQR controller implementation
- `mpc_controller.py` - MPC controller implementation

These modules are imported by Jupyter notebooks and provide:

- Angle wrapping utilities
- System discretization
- Reference trajectory generation
- Unified simulation framework
- Control algorithms with state estimation

## Development Workflow Summary

### Adding Features to Benji RT (Zephyr)

1. Modify source in `benji-rt/src/` or `benji-rt/inc/`
2. Update `prj.conf` if adding Zephyr subsystems
3. Build: `moon run benji-rt:build`
4. Flash: `moon run benji-rt:flash` (with hardware connected)
5. Monitor output via serial console

### Adding Features to Benji Compute (Yocto)

1. Modify `meta-benji/benji-devel.yaml` or add Yocto recipes
2. Setup: `moon run meta-benji:setup` (if adding new layers)
3. Build: `moon run meta-benji:build`
4. Flash to SD card/device (process TBD)

### Updating Documentation

1. Edit `.md` files in `doc/` or Jupyter notebooks in `doc/gnc/notebooks/`
2. Build: `moon run doc:build`
3. Preview: Open `doc/_build/html/index.html`
4. Commit changes (including executed notebooks if desired)

### Adding New ADRs

1. Create new file in `doc/adr/` following numbering scheme
2. Use template:
   ```markdown
   # N. Title
   
   Date: YYYY-MM-DD
   
   ## Status
   [Proposed|Accepted|Deprecated|Superseded]
   
   ## Context
   [What is the issue we're addressing?]
   
   ## Decision
   [What decision have we made?]
   
   ## Consequences
   [What are the results of this decision?]
   ```
3. ADRs are automatically included in documentation build

### Before Committing

1. Ensure all pre-commit hooks pass
2. Write conventional commit message
3. Commitizen hook will validate format
4. Push will trigger documentation build on `main` branch

## Quick Reference

```bash
# First time setup
just bootstrap              # Install pre-commit hooks
just setup                  # Initialize west modules
uv sync --locked            # Install Python deps

# Common tasks
just build benji-rt         # Build RT firmware
just build-all              # Build everything
moon run doc:build          # Build documentation
moon run benji-rt:flash     # Flash firmware

# Documentation
moon run doc:gen-notebooks  # Execute notebooks
moon run doc:build          # Build docs
moon run doc:clean          # Clean docs

# Check versions
moon --version              # Moon version
uv run west --version       # West version
uv --version                # uv version
```

## Additional Resources

- **Zephyr Documentation**: https://docs.zephyrproject.org/
- **Yocto Project**: https://www.yoctoproject.org/docs/
- **Moon Documentation**: https://moonrepo.dev/docs
- **uv Documentation**: https://github.com/astral-sh/uv
- **West Documentation**: https://docs.zephyrproject.org/latest/develop/west/index.html
- **Kas Documentation**: https://kas.readthedocs.io/

## Project-Specific Context

- **Development Focus**: Early stage, baseline hardware/software being established
- **Current Target Hardware**: Adafruit Feather STM32F405 (development board)
- **Future Plans**: Custom PCB with potentially different MCU
- **Compute Platform**: Raspberry Pi 5 with custom Yocto image
- **Base System**: AutonomOS (reused from previous projects)
- **Communication**: MCU ↔ SBC via high-bandwidth serial (protocol TBD)
- **Update Mechanism**: DFU for MCU, OTA (likely Mender) for compute platform
