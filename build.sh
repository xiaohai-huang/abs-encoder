#!/usr/bin/env bash
# Build the project with EIDE's bundled toolchain (unify_builder), no IDE needed.
# Usage:
#   ./build.sh                incremental build (Debug config)
#   ./build.sh Release        incremental build of the Release config
#   ./build.sh --rebuild [cfg]  clean rebuild (same as EIDE "Rebuild" task)
#   ./build.sh clean [cfg]    wipe <cfg>'s .obj (like EIDE "Clean")
#
# If the config is omitted, Debug is used; if Debug has never been built but
# exactly one other config has, that one is picked automatically.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ---- parse args: config name and mode can appear in any order ----
CONFIG=""
MODE="build"
for arg in "$@"; do
    case "$arg" in
        --rebuild|-r) MODE="rebuild" ;;
        clean)        MODE="clean" ;;
        *)            CONFIG="$arg" ;;
    esac
done

# ---- locate the EIDE extension (cl.eide-<version>) ----
EXT_DIR="$(ls -d "$HOME/.vscode/extensions"/cl.eide-* 2>/dev/null | sort -V | tail -1 || true)"
if [ -z "$EXT_DIR" ]; then
    echo "error: EIDE extension not found under $HOME/.vscode/extensions" >&2
    exit 1
fi
BUILDER="$EXT_DIR/res/tools/win32/unify_builder/unify_builder.exe"

# ---- resolve config -> build/<cfg>/builder.params ----
# only consider dirs that correspond to real targets in .eide/eide.yml
TARGETS="$(awk '/^targets:/{f=1; next} /^[^[:space:]]/{f=0} f && /^  [A-Za-z0-9_.-]+:/{sub(/^  */, ""); sub(/:.*/, ""); print}' .eide/eide.yml 2>/dev/null)"
AVAILABLE=""
for t in $TARGETS; do
    [ -f "build/$t/builder.params" ] && AVAILABLE="$AVAILABLE
build/$t/builder.params"
done
AVAILABLE="$(echo "$AVAILABLE" | sed '/^$/d' | sort)"
if [ -z "$CONFIG" ]; then
    CONFIG="Debug"
    if ! echo "$AVAILABLE" | grep -q "build/$CONFIG/builder.params" && [ -n "$AVAILABLE" ]; then
        # Debug never built; if there's exactly one other config, use it
        if [ "$(echo "$AVAILABLE" | wc -l)" -eq 1 ]; then
            CONFIG="$(basename "$(dirname "$AVAILABLE")")"
        fi
    fi
fi

PARAMS_FILE="build/$CONFIG/builder.params"
if [ ! -f "$PARAMS_FILE" ]; then
    echo "error: $PARAMS_FILE not found." >&2
    if [ -n "$AVAILABLE" ]; then
        echo "available configs:" >&2
        echo "$AVAILABLE" | sed 's|build/\(.*\)/builder.params|  \1|' >&2
    else
        echo "Open the project in VS Code and build once, or run the EIDE" >&2
        echo "'Generate Builder Params' command to create it." >&2
    fi
    exit 1
fi

WIN_PARAMS="$(cygpath -w "$PWD/$PARAMS_FILE" 2>/dev/null || echo "$PWD/$PARAMS_FILE")"

case "$MODE" in
    clean)
        echo ">> cleaning build/$CONFIG/.obj ..."
        rm -rf "build/$CONFIG/.obj"
        ;;
    rebuild)
        exec "$BUILDER" -p "$WIN_PARAMS" --rebuild
        ;;
    *)
        exec "$BUILDER" -p "$WIN_PARAMS"
        ;;
esac
