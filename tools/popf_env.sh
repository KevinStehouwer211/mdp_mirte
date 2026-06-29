# Point the PDDL planner node at the POPF binary built into this workspace.
# Sourced by pixi on environment activation (see [activation] in pixi.toml).
# Overridable: if POPF_BIN is already set, keep it.
if [ -z "${POPF_BIN:-}" ]; then
  export POPF_BIN="${PIXI_PROJECT_ROOT:-$PWD}/install/lib/popf/popf"
fi
