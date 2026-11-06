#!/usr/bin/env bash
# Render the architecture diagrams and install them where the site picks them up.
#
# The `.puml` sources live here so that a diagram is reviewable in a diff; the PNGs
# that the documentation includes are build products and are regenerated rather than
# hand-edited. `OUT_` is the documentation pipeline's prefix for "sphinx only": the
# build copies those from `docs/images/` into the sphinx image tree.
#
# PlantUML is not a dependency of the package. This script is a manual step, like the
# figure generator, because a documentation toolchain that runs inside the library's
# build would be a toolchain the library has to keep working.
set -euo pipefail

cd "$(dirname "$0")"
ROOT="$(cd ../../../.. && pwd)"

JAR="${PLANTUML_JAR:-/tmp/plantuml-new.jar}"
if command -v plantuml >/dev/null 2>&1; then
    RENDER=(plantuml)
elif [[ -f "$JAR" ]]; then
    RENDER=(java -jar "$JAR")
else
    echo "no plantuml: install it, or set PLANTUML_JAR to a recent plantuml.jar" >&2
    exit 1
fi

if "${RENDER[@]}" -version 2>&1 | grep -q '1\.2024'; then
    echo "that PlantUML is old enough to watermark every diagram; use a current one" >&2
    exit 1
fi

for source in ./*.puml; do
    name="$(basename "$source" .puml)"
    tmp="$(mktemp -d)"
    cp "$source" "$tmp/"
    "${RENDER[@]}" -tpng -scale 3 "$tmp/$name.puml"
    cp "$tmp/$name.png" "$ROOT/docs/images/OUT_arch_$name.png"
    rm -rf "$tmp"
    echo "docs/images/OUT_arch_$name.png"
done
