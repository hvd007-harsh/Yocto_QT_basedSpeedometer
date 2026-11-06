#!/usr/bin/env bash
# ==============================================================================
# Script: flash.sh
# Purpose: Safely flash Yocto WIC images to a target SD card/removable drive.
# ==============================================================================

set -Eeuo pipefail

# Configuration
IMAGE_PREFIX="rpi-secure-qt-image-raspberrypi0"
TARGET_MACHINE="raspberrypi0"

# Auto-detect image directory (checks standard Yocto deploy path or current dir)
YOCTO_DEPLOY_DIR="tmp/deploy/images/${TARGET_MACHINE}"
if [[ -d "$YOCTO_DEPLOY_DIR" ]]; then
    IMAGE_DIR="$YOCTO_DEPLOY_DIR"
else
    IMAGE_DIR="$(pwd)"
fi

# Ensure required system utilities exist
REQUIRED_CMDS=(lsblk findmnt dd sudo wipefs blockdev)
for cmd in "${REQUIRED_CMDS[@]}"; do
    command -v "$cmd" >/dev/null 2>&1 || { echo "ERROR: Required command missing: $cmd"; exit 1; }
done

SUDO="sudo"
[[ $EUID -eq 0 ]] && SUDO=""

echo "================================================="
echo " Yocto Image Flashing Utility"
echo " Image Search Path: $IMAGE_DIR"
echo "================================================="
echo

# Locate WIC image files (including symlinks, newest first)
mapfile -t IMAGES < <(find -L "$IMAGE_DIR" -maxdepth 1 -type f \
    \( -name "${IMAGE_PREFIX}*.wic.bz2" -o -name "${IMAGE_PREFIX}*.wic.gz" -o -name "${IMAGE_PREFIX}*.wic.zst" -o -name "${IMAGE_PREFIX}*.wic.xz" -o -name "${IMAGE_PREFIX}*.wic" \) \
    -printf '%T@ %p\n' 2>/dev/null | sort -nr | cut -d' ' -f2-)

if ((${#IMAGES[@]} == 0)); then
    echo "ERROR: No matching '${IMAGE_PREFIX}*.wic*' images found in $IMAGE_DIR."
    exit 1
fi

echo "Available Yocto Images (newest first):"
for i in "${!IMAGES[@]}"; do
    img_size=$(du -h "${IMAGES[$i]}" | cut -f1)
    img_name=$(basename "${IMAGES[$i]}")
    printf "  %d) %-65s (%s)\n" "$((i+1))" "$img_name" "$img_size"
done
echo

read -r -p "Select image number [1]: " choice
choice="${choice:-1}"
if ! [[ "$choice" =~ ^[0-9]+$ ]] || ((choice < 1 || choice > ${#IMAGES[@]})); then
    echo "ERROR: Invalid selection."
    exit 1
fi

IMAGE="$(readlink -f "${IMAGES[$((choice-1))]}")"
echo "Selected Image: $(basename "$IMAGE")"
echo

# Device Selection
echo "-------------------------------------------------"
echo "Removable Disks (Select a WHOLE drive, e.g., /dev/sdb or /dev/mmcblk0):"
lsblk -dpno NAME,SIZE,MODEL,TRAN,RM,TYPE | awk '$6=="disk" && $5=="1"'
echo
echo "Non-removable Disks (Do NOT select these):"
lsblk -dpno NAME,SIZE,MODEL,TRAN,RM,TYPE | awk '$6=="disk" && $5!="1"'
echo "-------------------------------------------------"
echo

read -r -p "Enter SD card device node (e.g. /dev/sdX or /dev/mmcblkX): " TARGET

if [[ -z "$TARGET" ]] || [[ ! -b "$TARGET" ]]; then
    echo "ERROR: Device '$TARGET' is invalid or not a block device."
    exit 1
fi

TARGET="$(readlink -f "$TARGET")"

# Validate block device type & removable status
if [[ "$(lsblk -dnro TYPE "$TARGET")" != "disk" ]]; then
    echo "ERROR: $TARGET is a partition. Please select a WHOLE disk."
    exit 1
fi

if [[ "$(lsblk -dnro RM "$TARGET" | tr -d ' ')" != "1" ]]; then
    echo "ERROR: Target device $TARGET is not marked as removable (RM=1). Refusing."
    exit 1
fi

# Critical Mountpoint Protection
CRITICAL_MOUNTS=(/ /boot /boot/efi /home "$IMAGE_DIR")
for mp in "${CRITICAL_MOUNTS[@]}"; do
    src="$(findmnt -nro SOURCE -T "$mp" 2>/dev/null || true)"
    [[ -n "$src" && -b "$src" ]] || continue
    parent="$(lsblk -s -nrpo NAME,TYPE "$src" | awk '$2=="disk"{print $1; exit}')"
    if [[ -n "$parent" && "$TARGET" == "$parent" ]]; then
        echo "ERROR: Refusing to format $TARGET — it backs active mountpoint '$mp' ($src)."
        exit 1
    fi
done

# Final Confirmation Prompt
echo
echo "================================================="
echo " TARGET DISK DETAILS:"
lsblk -dpno NAME,SIZE,MODEL,TRAN,RM "$TARGET"
echo "================================================="
echo "WARNING: ALL DATA ON $TARGET WILL BE PERMANENTLY ERASED!"
echo

TARGET_NAME="$(basename "$TARGET")"
read -r -p "To proceed, type exactly 'ERASE $TARGET_NAME': " confirm
if [[ "$confirm" != "ERASE $TARGET_NAME" ]]; then
    echo "Aborted by user."
    exit 1
fi

# Clean Unmount Process
echo
echo "[1/4] Unmounting mounted partitions on $TARGET..."
while read -r part; do
    [[ "$part" == "$TARGET" ]] && continue
    while read -r mp; do
        [[ -n "$mp" ]] || continue
        echo "  Unmounting partition $part from $mp..."
        $SUDO umount -f "$mp" || true
    done < <(findmnt -rn -S "$part" -o TARGET 2>/dev/null || true)
done < <(lsblk -lnpo NAME "$TARGET")

# Wipe partition structures
echo "[2/4] Wiping filesystem signatures & headers..."
$SUDO wipefs --all --force "$TARGET" >/dev/null 2>&1 || true

size_bytes="$($SUDO blockdev --getsize64 "$TARGET")"
echo "[3/4] Zeroing header and footer storage..."
$SUDO dd if=/dev/zero of="$TARGET" bs=1M count=8 conv=fsync status=none
last_mib=$(( (size_bytes / 1048576) - 8 ))
if (( last_mib > 0 )); then
    $SUDO dd if=/dev/zero of="$TARGET" bs=1M seek="$last_mib" count=8 conv=fsync status=none
fi
sync
$SUDO partprobe "$TARGET" 2>/dev/null || true

# Flashing Step
case "$IMAGE" in
    *.bz2) DECOMPRESS_CMD="bzcat" ;;
    *.gz)  DECOMPRESS_CMD="zcat" ;;
    *.zst) DECOMPRESS_CMD="zstdcat" ;;
    *.xz)  DECOMPRESS_CMD="xzcat" ;;
    *)     DECOMPRESS_CMD="cat" ;;
esac

command -v "$DECOMPRESS_CMD" >/dev/null 2>&1 || { echo "ERROR: Decompressor '$DECOMPRESS_CMD' not found in PATH."; exit 1; }

echo "[4/4] Decompressing and writing image ($DECOMPRESS_CMD) to $TARGET..."
if command -v pv >/dev/null 2>&1; then
    "$DECOMPRESS_CMD" "$IMAGE" | pv | $SUDO dd of="$TARGET" bs=4M conv=fsync status=none
else
    "$DECOMPRESS_CMD" "$IMAGE" | $SUDO dd of="$TARGET" bs=4M iflag=fullblock conv=fsync status=progress
fi

sync
$SUDO partprobe "$TARGET" 2>/dev/null || true

echo
echo "================================================="
echo " SUCCESS: Flash completed!"
echo "================================================="
lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINTS "$TARGET"
echo
echo "You can now safely eject $TARGET and plug it into your Raspberry Pi Zero."
