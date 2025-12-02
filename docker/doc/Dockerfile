# =============================================================================
# Benji Documentation Container
# =============================================================================
# Container for building Benji documentation with Sphinx.
# Supports both HTML and PDF output with slycot for control system notebooks.
#
# Uses uv for Python dependency management to respect uv.lock.
# =============================================================================

FROM python:3.13-slim-bookworm AS base

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# -----------------------------------------------------------------------------
# Stage 1: Build Python environment with doc dependencies (including slycot)
# -----------------------------------------------------------------------------
FROM base AS python-builder

# Install build dependencies for slycot (Fortran + BLAS/LAPACK)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gfortran \
    libblas-dev \
    liblapack-dev \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

WORKDIR /build

# Copy dependency files
COPY pyproject.toml uv.lock ./

# Create virtual environment and install all dependencies
# Using --frozen to respect the lock file exactly
# Note: Doc dependencies (sphinx, slycot, etc.) are in main dependencies
ENV VIRTUAL_ENV="/opt/venv" \
    PATH="/opt/venv/bin:$PATH"
RUN uv venv /opt/venv && \
    UV_PROJECT_ENVIRONMENT=/opt/venv uv sync --frozen --no-group dev

# -----------------------------------------------------------------------------
# Stage 2: Final runtime image
# -----------------------------------------------------------------------------
FROM base AS runtime

# Install runtime dependencies for slycot and documentation tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Slycot runtime dependencies
    libblas3 \
    liblapack3 \
    libgfortran5 \
    # Documentation dependencies
    graphviz \
    pandoc \
    # LaTeX for PDF builds (xelatex for Unicode support in notebooks)
    texlive-latex-recommended \
    texlive-latex-extra \
    texlive-fonts-recommended \
    texlive-fonts-extra \
    texlive-xetex \
    latexmk \
    # Free fonts required by xelatex/polyglossia
    fonts-freefont-otf \
    # Make for running latexmk from sphinx
    make \
    && rm -rf /var/lib/apt/lists/*

# Copy uv
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

# Copy Python virtual environment (includes sphinx, slycot, etc.)
COPY --from=python-builder /opt/venv /opt/venv

# Set environment variables
ENV PATH="/opt/venv/bin:$PATH" \
    VIRTUAL_ENV="/opt/venv"

# Working directory for documentation builds
WORKDIR /workspace

# Default to sphinx-build
ENTRYPOINT ["sphinx-build"]
CMD ["-M", "html", ".", "_build"]
