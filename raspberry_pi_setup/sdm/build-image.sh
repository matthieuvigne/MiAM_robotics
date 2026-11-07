#!/bin/bash
# Customize a compressed image using sdm; keep the original archive.
set -euo pipefail
umask 077

die() { echo "Error: $*" >&2; exit 1; }
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    echo "Usage: $0 /path/to/secrets.conf /path/to/original.img.gz"
    echo 'Also accepts .img.xz. Creates original_miam.img.gz beside the input.'
    exit 0
fi
[[ $# == 2 ]] || die "Usage: $0 SECRETS_CONF IMAGE.img.gz"
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
secrets=$(realpath -e -- "$1")
archive=$(realpath -e -- "$2")
[[ -f $secrets && -f $archive ]] || die 'Secrets and image must be regular files'
[[ -f $script_dir/robot.conf ]] || die "Missing $script_dir/robot.conf"
case "$archive" in
    *.img.gz) stem=${archive%.img.gz}; decompress=(gzip -dc) ;;
    *.img.xz) stem=${archive%.img.xz}; decompress=(xz -dc) ;;
    *) die 'Expected an .img.gz or .img.xz file' ;;
esac
output="${stem}_miam.img.gz"
[[ ! -e $output && ! -L $output ]] || die "Output already exists: $output"
sdm_command=$(command -v sdm) || die 'Install sdm first (see README.md)'
privileged=()
if (( EUID != 0 )); then
    privileged=(sudo)
    sudo -v
fi

# Keep the large working image on the same disk as the input. sdm does not
# consistently quote its paths, so use fixed filenames in a temporary directory.
work=$(mktemp -d "$(dirname -- "$archive")/.miam-image.XXXXXXXX")
trap 'rm -rf -- "$work"' EXIT
[[ $work != *[[:space:]]* ]] || die 'sdm needs an image directory without spaces'
install -m 644 "$script_dir/robot.conf" "$work/robot.conf"
install -m 600 "$secrets" "$work/secrets.conf"
install -m 755 "$script_dir/configure-image.sh" "$work/configure-image.sh"

echo "Decompressing $archive"
"${decompress[@]}" -- "$archive" > "$work/robot.img"
echo 'Configuring the image with sdm'
"${privileged[@]}" "$sdm_command" --customize --batch --extend --autologin --restart \
    --regen-ssh-host-keys --nowait-timesync \
    --cscript "$work/configure-image.sh" --csrc "$work" "$work/robot.img"
echo "Compressing $output"
gzip -1 -c -- "$work/robot.img" > "$work/result.img.gz"
# Publish only a complete archive; ln also refuses to overwrite an existing file.
ln -- "$work/result.img.gz" "$output"
echo "Created $output"
