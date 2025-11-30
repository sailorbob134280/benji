# Suggested Commands

## Important: Always Use `uv run` Prefix
All Python tools must be run via `uv run` to use the project's virtual environment:
```bash
uv run west --version
uv run pytest
uv run jupyter notebook
```

## Initial Setup
```bash
# Install pre-commit hooks (required first time)
just bootstrap

# Set up all projects (updates west modules)
just setup

# Install Python dependencies manually if needed
uv sync --locked --all-extras --dev
```

## Building

```bash
# Build specific target
just build <target>
# Example: just build benji-rt

# Build all projects
just build-all

# Clean all projects
just clean
```

## Benji RT (Zephyr/Embedded)

```bash
# Build the RT application
moon run benji-rt:build

# Flash to target device (via J-Link)
moon run benji-rt:flash

# Direct west commands
uv run west build benji-rt
uv run west flash --runner jlink
```

## Benji Compute (Yocto/Linux)

```bash
# Setup build environment
moon run meta-benji:setup

# Build the compute platform image (takes hours!)
moon run meta-benji:build

# Clean build artifacts
moon run meta-benji:clean

# Complete wipe (WARNING: removes sources/ and build/)
moon run meta-benji:spotless
```

## Documentation

```bash
# Execute Jupyter notebooks
moon run doc:gen-notebooks

# Build documentation
moon run doc:build

# Clean documentation build
moon run doc:clean

# View built docs: open doc/_build/html/index.html
```

## Testing (when tests are added)

```bash
uv run pytest
```

## Utility Commands

```bash
# List Just recipes
just help

# Check versions
moon --version
uv run west --version
uv --version
```

## System Commands (Linux)
Standard Linux commands: `git`, `ls`, `cd`, `grep`, `find`, etc.
