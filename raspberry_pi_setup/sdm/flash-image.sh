#!/bin/bash
# Unpack a MiAM image and flash it with sdm, expanding to fill the SD card.
set -euo pipefail

die() { echo "Error: $*" >&2; exit 1; }
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    echo "Usage: $0 /path/to/robot_miam.img.gz /dev/sdX"
    echo 'Also accepts an unpacked .img. Keeps the archive and unpacked image.'
    exit 0
fi
[[ $# == 2 ]] || die "Usage: $0 IMAGE.img.gz SD_CARD_DEVICE"
archive=$(realpath -e -- "$1")
device=$(realpath -e -- "$2")
[[ -f $archive ]] || die 'Image must be a regular file'
[[ -b $device ]] || die "Not a block device: $device"
decompress=false
gunzip_options=(--keep)
case "$archive" in
    *.img.gz)
        image=${archive%.gz}
        decompress=true
        ;;
    *.img) image=$archive ;;
    *) die 'Expected an .img.gz or .img file' ;;
esac
[[ $image != *[[:space:]]* ]] || die 'sdm needs an image path without spaces'
sdm_command=$(command -v sdm) || die 'Install sdm first (see README.md)'
if $decompress && [[ -e $image || -L $image ]]; then
    [[ -f $image && ! -L $image ]] || die "Unpacked image must be a regular file, not a symlink: $image"
    echo "Unpacked image already exists: $image"
    echo 'It may belong to an older build.'
    echo 'a) Flash the existing .img'
    echo 'b) Overwrite the .img from this archive, then flash'
    echo 'c) Cancel (default)'
    while true; do
        read -r -p 'Choice [c]: ' choice || choice=c
        case "$choice" in
            a|A) decompress=false; break ;;
            b|B) gunzip_options+=(--force); break ;;
            c|C|'') echo 'Cancelled.'; exit 0 ;;
            *) echo 'Choose a, b or c.' ;;
        esac
    done
fi
privileged=()
if (( EUID != 0 )); then
    privileged=(sudo)
    sudo -v
fi

if $decompress; then
    echo "Decompressing $archive"
    gunzip "${gunzip_options[@]}" -- "$archive"
fi
echo "Flashing $image to $device"
# Buffered reads avoid direct-I/O incompatibilities; fsync flushes the copy.
"${privileged[@]}" "$sdm_command" --burn "$device" --expand-root \
    --ddsw 'bs=4M conv=fsync' "$image"
