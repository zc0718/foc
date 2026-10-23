FROM mcr.microsoft.com/devcontainers/base:ubuntu-24.04

# Pin clang-format==23.1.0 to match the native quality gate
# (.github/workflows/security-linters.yml); apt ships v18 on ubuntu-24.04.
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc g++ cmake ninja-build clang clang-tidy \
    python3 python3-pip python3-venv nodejs npm \
    doxygen graphviz lcov jq \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir --break-system-packages conan clang-format==23.1.0 \
    && conan profile detect --force

# Run as the non-root devcontainer user (avoid default-root image finding).
USER vscode
