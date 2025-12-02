# =============================================================================
# Benji RT Development Container
# =============================================================================
# Container for Zephyr RTOS builds with west and Zephyr SDK 0.17.0
#
# Uses uv for Python dependency management to respect uv.lock.
# =============================================================================

ARG ZEPHYR_SDK_VERSION=0.17.0

FROM python:3.13-slim-bookworm AS base

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# -----------------------------------------------------------------------------
# Stage 1: Download Zephyr SDK
# -----------------------------------------------------------------------------
FROM base AS sdk-downloader

ARG ZEPHYR_SDK_VERSION

RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    xz-utils \
    cmake \
    file \
    python3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /sdk

# Download and extract Zephyr SDK (minimal install with ARM toolchain only)
RUN wget -q https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZEPHYR_SDK_VERSION}/zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64_minimal.tar.xz && \
    tar xf zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64_minimal.tar.xz && \
    rm zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64_minimal.tar.xz && \
    cd zephyr-sdk-${ZEPHYR_SDK_VERSION} && \
    ./setup.sh -t arm-zephyr-eabi -h -c

# -----------------------------------------------------------------------------
# Stage 2: Clone Zephyr and modules
# -----------------------------------------------------------------------------
FROM base AS zephyr-sources

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install uv and west for cloning
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

# Set up west workspace structure matching west.yaml's self.path: benji
WORKDIR /zephyr-workspace/benji

# Copy project files for west initialization
COPY west.yml ./west.yml
COPY pyproject.toml uv.lock ./

# Install west via uv
ENV VIRTUAL_ENV="/opt/venv" \
    PATH="/opt/venv/bin:$PATH"
RUN uv venv /opt/venv && \
    uv pip install west

# Initialize west and update all modules
# This bakes Zephyr sources into the image
# West will create deps/ relative to workspace root (/zephyr-workspace/deps/)
WORKDIR /zephyr-workspace
RUN west init -l benji && \
    west update --narrow --fetch-opt=--depth=1

# -----------------------------------------------------------------------------
# Stage 3: Build Python environment with west and Zephyr tools
# -----------------------------------------------------------------------------
FROM base AS python-builder

COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

# Create virtual environment and install only west + Zephyr dependencies
# We don't need the full project dependencies (sphinx, slycot, etc.) for RT builds
ENV VIRTUAL_ENV="/opt/venv" \
    PATH="/opt/venv/bin:$PATH"
RUN uv venv /opt/venv && \
    uv pip install west pyelftools pyyaml packaging

# -----------------------------------------------------------------------------
# Stage 4: Final runtime image
# -----------------------------------------------------------------------------
FROM base AS runtime

ARG ZEPHYR_SDK_VERSION

# Install Zephyr build runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    ninja-build \
    device-tree-compiler \
    git \
    gperf \
    ccache \
    dfu-util \
    make \
    gcc \
    g++ \
    libsdl2-dev \
    libmagic1 \
    file \
    && rm -rf /var/lib/apt/lists/*

# Copy uv
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

# Copy Zephyr SDK from downloader stage
COPY --from=sdk-downloader /sdk/zephyr-sdk-${ZEPHYR_SDK_VERSION} /opt/zephyr-sdk

# Create symlink to satisfy SDK binaries with hardcoded interpreter paths
# SDK binaries expect /sdk/zephyr-sdk-0.17.0/sysroots/.../ld-linux-x86-64.so.2
RUN mkdir -p /sdk && ln -s /opt/zephyr-sdk /sdk/zephyr-sdk-${ZEPHYR_SDK_VERSION}

# Copy Zephyr workspace (including .west config and deps)
COPY --from=zephyr-sources /zephyr-workspace/.west /opt/zephyr-workspace/.west
COPY --from=zephyr-sources /zephyr-workspace/deps /opt/zephyr-workspace/deps

# Copy Python virtual environment (includes west)
COPY --from=python-builder /opt/venv /opt/venv

# Set environment variables
ENV PATH="/opt/venv/bin:$PATH" \
    VIRTUAL_ENV="/opt/venv" \
    ZEPHYR_SDK_INSTALL_DIR="/opt/zephyr-sdk" \
    ZEPHYR_BASE="/opt/zephyr-workspace/deps/zephyr" \
    ZEPHYR_TOOLCHAIN_VARIANT="zephyr"

# Working directory for application builds
WORKDIR /workspace

CMD ["/bin/bash"]
