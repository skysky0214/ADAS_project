#!/usr/bin/env bash
set +u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2

export EGO_SOURCE="${EGO_SOURCE:-topic}"
exec bash tools/run_live_ego_comp.sh "$@"
