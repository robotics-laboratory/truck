#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USE_SIM_TIME="false"
USE_RVIZ="false"
TRUCK_LAUNCH="truck.yaml"

usage() {
    cat <<'EOF'
Usage:
  scripts/run_localizer_stack.sh [--use-sim-time true|false] [--use-rviz true|false] [--truck-launch FILE]

Examples:
  scripts/run_localizer_stack.sh
  scripts/run_localizer_stack.sh --use-sim-time true
  scripts/run_localizer_stack.sh --truck-launch truck.yaml --use-rviz false
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --use-sim-time)
            USE_SIM_TIME="${2:?missing value for --use-sim-time}"
            shift 2
            ;;
        --use-rviz)
            USE_RVIZ="${2:?missing value for --use-rviz}"
            shift 2
            ;;
        --truck-launch)
            TRUCK_LAUNCH="${2:?missing value for --truck-launch}"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

declare -a PIDS=()

cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM
    if [[ ${#PIDS[@]} -gt 0 ]]; then
        echo "Stopping localizer stack..."
        kill "${PIDS[@]}" 2>/dev/null || true
        wait "${PIDS[@]}" 2>/dev/null || true
    fi
    exit "$exit_code"
}

trap cleanup EXIT INT TERM

run_component() {
    local name="$1"
    shift

    (
        cd "$ROOT_DIR"
        source "$ROOT_DIR/scripts/rosenv.sh"
        exec "$@"
    ) > >(stdbuf -oL sed "s/^/[$name] /") \
      2> >(stdbuf -oL sed "s/^/[$name] /" >&2) &

    PIDS+=("$!")
}

echo "Starting truck stack..."
run_component truck ros2 launch truck "$TRUCK_LAUNCH"

sleep 2

echo "Starting localizer..."
run_component localizer ros2 launch localizer localizer_launch.py "use_rviz:=${USE_RVIZ}" "use_sim_time:=${USE_SIM_TIME}"

sleep 1

echo "Starting foxglove localization proxy..."
run_component proxy python3 "$ROOT_DIR/foxglove_localization_proxy.py"

echo "Localizer stack is running. Press Ctrl-C to stop all processes."

wait -n "${PIDS[@]}"
echo "A process exited. Shutting down the rest..."
