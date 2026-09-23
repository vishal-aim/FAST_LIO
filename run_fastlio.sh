#!/usr/bin/env bash
set -e

# ==============================================================================
# FAST-LIO Standalone MCAP Runner Script (via Docker + CUDA)
# ==============================================================================

function show_help() {
  cat << 'EOF'
Usage: ./run_fastlio.sh [script_options] [fastlio_options]

Script Options:
  --rebuild             Rebuild the docker image before running
  -h, --help            Show this help message

FAST-LIO Options (passed to fastlio_mcap):
  -i, --input <file>    Path to input MCAP file (e.g. /home/vishal/data/recording.mcap)
  -o, --output <dir>    Output directory for trajectory (default: ./results)
  -c, --config <file>   Configuration YAML file (default: config/fastlio_ouster.yaml)
  -d, --duration <sec>  Maximum duration in seconds to process (default: full file)
  -s, --start <sec>     Start offset in seconds from beginning of recording
  -r, --rate <float>    Playback rate multiplier (1.0 = real-time, 0 = max speed)
  --lidar <topic>       LiDAR topic (default: /ouster/points)
  --imu <topic>         IMU topic (default: /ouster/imu)
  --headless            Run headless (default)

Examples:
  # Run on target dataset:
  ./run_fastlio.sh -i /home/vishal/data/2026-08-first/aimbag_2026_07_17-23_28_01_concat.mcap -o ~/data/fastlio_results
EOF
  exit 0
}

IMAGE_NAME="fastlio:mcap"
REBUILD=0
INPUT_FILE=""
OUTPUT_DIR=""
FASTLIO_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rebuild)
      REBUILD=1
      shift
      ;;
    -h|--help)
      show_help
      ;;
    -i|--input)
      INPUT_FILE="$2"
      shift 2
      ;;
    -o|--output)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      FASTLIO_ARGS+=("$1")
      shift
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Interactive fallback if no input provided
if [[ -z "$INPUT_FILE" ]] && command -v zenity &>/dev/null && [[ -n "$DISPLAY" ]]; then
  INPUT_FILE=$(zenity --file-selection --title="Select Input MCAP File" --file-filter="MCAP files (*.mcap) | *.mcap" 2>/dev/null || true)
fi

if [[ -z "$INPUT_FILE" ]]; then
  echo "Error: Input MCAP file is required (-i /path/to/file.mcap)"
  echo "Run ./run_fastlio.sh --help for usage details."
  exit 1
fi

if [[ ! -f "$INPUT_FILE" ]]; then
  echo "Error: File does not exist: $INPUT_FILE"
  exit 1
fi

INPUT_ABS="$(realpath "$INPUT_FILE")"
INPUT_DIR="$(dirname "$INPUT_ABS")"
INPUT_BASENAME="$(basename "$INPUT_ABS")"

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="$(pwd)/fastlio_results"
fi
mkdir -p "$OUTPUT_DIR"
OUTPUT_ABS="$(realpath "$OUTPUT_DIR")"

# 2. Build or Rebuild Docker Image if needed
if [[ "$REBUILD" -eq 1 ]] || ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
  echo "Building Docker image ($IMAGE_NAME)..."
  docker build -t "$IMAGE_NAME" "$SCRIPT_DIR"
fi

# 3. GPU Detection
DOCKER_GPU_FLAGS=""
if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
  DOCKER_GPU_FLAGS="--gpus all"
fi

# 4. Run Docker Container
DOCKER_ARGS=(
  --rm
  $([[ -t 0 ]] && echo "-it" || echo "-i")
  $DOCKER_GPU_FLAGS
  --user "$(id -u):$(id -g)"
  --ipc=host
  --net=host
  -v "$INPUT_DIR:/data:ro"
  -v "$OUTPUT_ABS:/output"
  -v "$SCRIPT_DIR/config:/opt/fastlio/config:ro"
)

echo "=========================================================="
echo " Running FAST-LIO Standalone MCAP via Docker"
echo " Image : $IMAGE_NAME"
echo " Input : $INPUT_ABS"
echo " Output: $OUTPUT_ABS"
echo "=========================================================="

exec docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME" \
  -i "/data/$INPUT_BASENAME" \
  -o /output \
  "${FASTLIO_ARGS[@]}"
