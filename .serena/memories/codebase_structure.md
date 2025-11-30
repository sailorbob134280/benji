# Codebase Structure

```
benji/
├── benji-rt/              # Zephyr RTOS real-time control application
│   ├── src/               # C/C++ source files
│   ├── inc/               # Header files
│   ├── boards/            # Board-specific overlays
│   ├── build/             # Build output (generated)
│   ├── CMakeLists.txt     # CMake build configuration
│   ├── prj.conf           # Zephyr project configuration
│   └── moon.yml           # Moon task definitions
│
├── meta-benji/            # Yocto layer for compute platform
│   ├── sources/           # Kas-managed sources (generated)
│   ├── build/             # Build output (generated)
│   ├── benji-devel.yaml   # Kas configuration for development image
│   ├── secrets.yaml.tmpl  # Template for secrets (DON'T commit secrets.yaml)
│   └── moon.yml           # Moon task definitions
│
├── doc/                   # Sphinx documentation
│   ├── adr/               # Architectural Decision Records
│   ├── gnc/               # Guidance, Navigation, Control docs
│   │   └── notebooks/     # Jupyter notebooks with algorithms
│   ├── architecture/      # Architecture documentation
│   ├── hardware/          # Hardware documentation
│   ├── projects/          # Project-specific docs
│   ├── _build/            # Built documentation (generated)
│   ├── _static/           # Static assets
│   ├── _templates/        # Sphinx templates
│   ├── conf.py            # Sphinx configuration
│   ├── index.md           # Documentation index
│   └── moon.yml           # Moon task definitions
│
├── deps/                  # West-managed dependencies (Zephyr, etc.)
├── .west/                 # West configuration
├── .moon/                 # Moon cache and workspace config
├── .venv/                 # Python virtual environment (generated)
├── .github/               # GitHub Actions workflows
├── .serena/               # Serena configuration
│
├── pyproject.toml         # Python project dependencies
├── uv.lock                # Locked dependency versions
├── west.yaml              # West manifest
├── Justfile               # Just command recipes
├── moon.yml               # Root Moon task definitions
├── .cz.toml               # Commitizen configuration
├── .pre-commit-config.yaml # Pre-commit hooks config
├── .python-version        # Python version specification
├── .gitignore             # Git ignore rules
├── README.md              # Project readme
└── AGENTS.md              # Developer guide (comprehensive)
```

## Moon Workspace Projects

Defined in `.moon/workspace.yml`:
- `.` - Root project
- `benji-rt` - Zephyr RT application
- `doc` - Documentation
- `meta-benji` - Yocto compute platform

Each project has its own `moon.yml` with task definitions.

## Key Files

- **AGENTS.md**: Comprehensive developer guide (most important reference)
- **pyproject.toml**: Python dependencies and project metadata
- **west.yaml**: West manifest for Zephyr dependencies
- **Justfile**: User-friendly command shortcuts
- **moon.yml** (root): Root-level Moon tasks (setup)
