# Task Completion Checklist

## Before Committing

1. **Run pre-commit hooks** (if not using `just bootstrap`)
   ```bash
   pre-commit run --all-files
   ```

2. **Write conventional commit message**
   - Format: `<type>(<scope>): <subject>`
   - Types: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`, `build`, `ci`
   - Commitizen hook will validate format

3. **Ensure no secrets are committed**
   - Check `meta-benji/secrets.yaml` is NOT staged (should be in .gitignore)

## After Code Changes

### For Benji RT (Zephyr) changes:
```bash
moon run benji-rt:build
```

### For Documentation changes:
```bash
moon run doc:build
# Preview: open doc/_build/html/index.html
```

### For Python code changes:
```bash
# When tests exist:
uv run pytest

# Linting (ruff):
uv run ruff check .
uv run ruff format .

# Type checking:
uv run mypy .
```

### For Benji Compute (Yocto) changes:
```bash
moon run meta-benji:build
# Note: This is very slow (hours)
```

## Pre-commit Hooks (automated)

After `just bootstrap`, these run automatically:

1. **Pre-commit stage**:
   - Trailing whitespace removal
   - End of file fixer
   - Large file check

2. **Commit-msg stage**:
   - Commitizen validation

3. **Pre-push stage**:
   - Commitizen branch validation

## CI/CD

Push to `main` branch triggers:
1. Documentation build
2. Deploy to GitHub Pages

## Important Notes

- Always use `uv run` prefix for Python tools
- Moon task dependencies are handled automatically
- Yocto builds don't use Moon caching (BitBake handles it)
- Documentation build order: gen-notebooks → build
