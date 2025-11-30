# Code Style and Conventions

## Python Code (GNC Notebooks, tooling)

- **Module names**: lowercase with underscores (`simulation.py`, `kalman.py`)
- **Function names**: lowercase with underscores (`angle_wrap`, `discretize_system`)
- **Class names**: PascalCase (`TankDriveSimulation`, `KalmanFilter`)
- **Constants**: UPPERCASE with underscores
- **Docstrings**: Google style with Args/Returns sections
- **Type hints**: Used where appropriate
- **Python version**: 3.13+
- **Linter**: ruff (version 0.11.11 pinned)
- **Type checker**: mypy

## C/C++ Code (Benji RT)

- **Files**: lowercase with underscores (`main.c`)
- **Macros**: UPPERCASE with underscores (`HELLO_INTERVAL_MS`)
- **Functions**: Follows Zephyr conventions (lowercase with underscores)
- **Indentation**: Spaces (verify with existing code)
- **C++ Standard**: C++23 enabled (`CONFIG_STD_CPP23=y`)
- **Formatter**: clang-format

## Configuration Files

- **Zephyr configs**: `CONFIG_<SUBSYSTEM>_<OPTION>` format in `prj.conf`
- **Moon tasks**: lowercase with hyphens (`gen-notebooks`, `build-all`)
- **Just recipes**: lowercase with hyphens (mirrors moon tasks)

## Git Commit Messages

Uses **Commitizen** with conventional commits:
```
<type>(<scope>): <subject>

<body>

<footer>
```

**Types**: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`, `build`, `ci`

**Examples**:
```
build: use uv to run things instead of relying on the venv
docs(adr): add ADRs discussing the high-level domain split
ci(doc): build and publish docs to github pages
```

## Documentation

- **Formats**: Markdown (MyST dialect), reStructuredText
- **Theme**: sphinx_rtd_theme
- **Math**: Dollar syntax (`$inline$`, `$$display$$`)
- **Jupyter notebooks**: Title Case names with spaces
- **ADRs**: Standard format with Status/Context/Decision/Consequences

## Naming for GNC Variables

| Symbol | Name | Unit |
|--------|------|------|
| $L$ | Wheelbase | m |
| $\tau$ | Time constant for motor/drivetrain lag | s |
| $v_0$ | Speed about which to linearize | m/s |
| $x, y$ | Position in global frame | m |
| $\theta$ | Heading angle | rad |
| $v_l, v_r$ | Left and right wheel speeds | m/s |
