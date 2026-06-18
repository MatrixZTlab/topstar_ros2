#!/usr/bin/env bash
set -euo pipefail

# Create a clean redeployable archive of this workspace.
# Usage:
#   ./zip_redeploy.sh
#   ./zip_redeploy.sh custom_name.zip

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
WORKSPACE_PARENT="$(dirname "$SCRIPT_DIR")"

DATE_TAG="$(date +%Y%m%d)"
DEFAULT_ZIP="topstar_ros2_redeploy_${DATE_TAG}.zip"
ZIP_NAME="${1:-$DEFAULT_ZIP}"
ROS2_DIR_NAME="$(basename "$SCRIPT_DIR")"
TOPSTAR_MUJOCO_DIR="${TOPSTAR_MUJOCO_DIR:-${WORKSPACE_PARENT}/topstar_mujoco}"
TOPSTAR_H2_DIR="${TOPSTAR_H2_DIR:-${WORKSPACE_PARENT}/topstar_h2}"

if [[ "$ZIP_NAME" != *.zip ]]; then
  ZIP_NAME="${ZIP_NAME}.zip"
fi

ROS2_ITEMS=(
  README.md
  setup.sh
  setup_default.sh
  setup_local.sh
  version.txt
  zip_redeploy.sh
  example
  cyclonedds_ws
)

# Include all markdown docs in repository root (README + any additional docs).
shopt -s nullglob
for md_file in "$SCRIPT_DIR"/*.md; do
  md_basename="$(basename "$md_file")"
  if [[ ! " ${ROS2_ITEMS[*]} " =~ " ${md_basename} " ]]; then
    ROS2_ITEMS+=("$md_basename")
  fi
done
shopt -u nullglob

MUJOCO_ITEMS=(
  simulate
  simulate_python
  topstar_robots
)

H2_ITEMS=(
  h2_model
)

for item in "${ROS2_ITEMS[@]}"; do
  if [[ ! -e "$SCRIPT_DIR/$item" ]]; then
    echo "Missing required path in ${ROS2_DIR_NAME}: $item" >&2
    exit 1
  fi
done

# Include root src if present in the future.
if [[ -d "$SCRIPT_DIR/src" ]]; then
  ROS2_ITEMS+=(src)
fi

copy_tree() {
  local source_root="$1"
  local source_rel="$2"
  local dest_root="$3"
  shift 3

  if [[ ! -e "$source_root/$source_rel" ]]; then
    echo "Missing required path: $source_root/$source_rel" >&2
    exit 1
  fi

  mkdir -p "$dest_root"
  rsync "$@" "$source_root/$source_rel" "$dest_root/"
}

materialize_h1_mesh_links() {
  local staged_h1_dir="$1"
  local fallback_mesh_dir="$2"
  local unresolved=0

  [[ -d "$staged_h1_dir" ]] || return 0

  while IFS= read -r link_path; do
    local mesh_name
    mesh_name="$(basename "$link_path")"

    if [[ -f "$fallback_mesh_dir/$mesh_name" ]]; then
      rm -f "$link_path"
      cp "$fallback_mesh_dir/$mesh_name" "$link_path"
    else
      echo "ERROR: unresolved mesh dependency: $mesh_name" >&2
      unresolved=1
    fi
  done < <(find "$staged_h1_dir" -maxdepth 1 -type l)

  if (( unresolved )); then
    echo "ERROR: topstar_mujoco/topstar_robots/h1 still contains external mesh links that are not available locally." >&2
    exit 1
  fi
}

stage_dir="$(mktemp -d)"
trap 'rm -rf "$stage_dir"' EXIT

ros2_stage_root="$stage_dir/$ROS2_DIR_NAME"
mkdir -p "$ros2_stage_root"

for item in "${ROS2_ITEMS[@]}"; do
  copy_tree "$SCRIPT_DIR" "$item" "$ros2_stage_root" \
    -aL \
    --exclude="build/" \
    --exclude="install/" \
    --exclude="log/" \
    --exclude=".cache/" \
    --exclude=".git/" \
    --exclude=".claude/" \
    --exclude=".venv/" \
    --exclude="__pycache__/" \
    --exclude="*.pyc"
done

included_roots=("$ROS2_DIR_NAME")

if [[ -d "$TOPSTAR_MUJOCO_DIR" ]]; then
  mujoco_stage_root="$stage_dir/$(basename "$TOPSTAR_MUJOCO_DIR")"
  mkdir -p "$mujoco_stage_root"
  copy_tree "$TOPSTAR_MUJOCO_DIR" readme.md "$mujoco_stage_root" -aL
  for item in simulate simulate_python; do
    copy_tree "$TOPSTAR_MUJOCO_DIR" "$item" "$mujoco_stage_root" \
      -aL \
      --exclude="build/" \
      --exclude="dist/" \
      --exclude="temp/" \
      --exclude=".cache/" \
      --exclude="__pycache__/" \
      --exclude="*.pyc"
  done
  copy_tree "$TOPSTAR_MUJOCO_DIR" topstar_robots "$mujoco_stage_root" -a
  included_roots+=("$(basename "$TOPSTAR_MUJOCO_DIR")")
else
  echo "WARNING: topstar_mujoco not found at $TOPSTAR_MUJOCO_DIR; skipping MuJoCo dependency bundle." >&2
fi

if [[ -d "$TOPSTAR_H2_DIR" ]]; then
  h2_stage_root="$stage_dir/$(basename "$TOPSTAR_H2_DIR")"
  mkdir -p "$h2_stage_root"
  for item in "${H2_ITEMS[@]}"; do
    copy_tree "$TOPSTAR_H2_DIR" "$item" "$h2_stage_root" \
      -aL \
      --exclude="urdf/h2_abs.urdf" \
      --exclude="build/" \
      --exclude="dist/" \
      --exclude=".cache/" \
      --exclude="__pycache__/" \
      --exclude="*.pyc"
  done
  included_roots+=("$(basename "$TOPSTAR_H2_DIR")")
else
  echo "WARNING: topstar_h2 not found at $TOPSTAR_H2_DIR; skipping H2 model bundle." >&2
fi

rm -f "$ZIP_NAME"

(
  cd "$stage_dir"
  zip -ry "$SCRIPT_DIR/$ZIP_NAME" "${included_roots[@]}"
)

echo
echo "Created: $SCRIPT_DIR/$ZIP_NAME"
printf 'Included roots:\n'
for root in "${included_roots[@]}"; do
  printf '  %s\n' "$root"
done
sha256sum "$ZIP_NAME"
