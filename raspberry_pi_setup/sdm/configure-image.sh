#!/bin/bash
# sdm custom phase script. See README.md for the command to run.
set -eo pipefail
umask 077
phase=${1:-}
[[ -v SDMPT && -f $SDMPT/etc/sdm/cparams ]] || {
    echo 'Run this script with sdm --cscript (see README.md).' >&2; exit 1;
}
# shellcheck source=/dev/null
source "$SDMPT/etc/sdm/sdm-readparams" ""
assets="$SDMPT/etc/sdm/assets/miam"

if [[ $phase == 0 ]]; then
    [[ -n $SDMPT && $SDMPT != / ]] || { echo 'Missing image mount point' >&2; exit 1; }
    : "${csrc:?Pass the settings directory using sdm --csrc}"
    [[ -f $csrc/robot.conf && -f $csrc/secrets.conf ]] || {
        echo 'Provide robot.conf and secrets.conf (see README.md).' >&2; exit 1;
    }
    install -d -m 700 "$assets"
    install -m 644 "$csrc/robot.conf" "$assets/robot.conf"
    install -m 600 "$csrc/secrets.conf" "$assets/secrets.conf"
elif [[ $phase != 1 && $phase != post-install ]] || [[ -n $SDMPT ]]; then
    echo 'Unexpected sdm phase or mount point' >&2; exit 1
fi
# Both settings files are trusted Bash code. Keep secrets out of sdm's logs.
# shellcheck source=/dev/null
source "$assets/robot.conf"
ROBOT_PASSWORD=''
HOTSPOT_SSID=''
HOTSPOT_PASSWORD=''
# shellcheck source=/dev/null
source "$assets/secrets.conf"
: "${ROBOT_PASSWORD:?Set ROBOT_PASSWORD in secrets.conf}"
: "${HOTSPOT_SSID:?Set HOTSPOT_SSID in secrets.conf}"
: "${HOTSPOT_PASSWORD:?Set HOTSPOT_PASSWORD in secrets.conf}"
[[ $ROBOT_USER =~ ^[a-z_][a-z0-9_-]{0,30}$ && $ROBOT_USER != root ]] || {
    echo 'Invalid ROBOT_USER' >&2; exit 1;
}
[[ $ROBOT_PASSWORD != *$'\n'* && $ROBOT_PASSWORD != *:* ]] || {
    echo 'ROBOT_PASSWORD cannot contain a newline or colon' >&2; exit 1;
}
[[ ${#HOTSPOT_PASSWORD} -ge 8 && ${#HOTSPOT_PASSWORD} -le 63 ]] || {
    echo 'HOTSPOT_PASSWORD must be 8 to 63 characters' >&2; exit 1;
}
case "$SCREEN_ROTATION" in normal|inverted|left|right) ;; *) echo 'Invalid SCREEN_ROTATION' >&2; exit 1 ;; esac

if [[ $phase == 0 ]]; then
    # Tell sdm which account to use for desktop autologin after its first boot.
    # shellcheck disable=SC2034
    myuser=$ROBOT_USER
    writeconfig
fi
# Use sdm's localization support for locale generation and Wi-Fi country setup.
runoneplugin L10n "$phase" "locale=$LOCALE|keymap=$KEYMAP|timezone=$TIMEZONE|wificountry=$WIFI_COUNTRY"
[[ $phase != 0 ]] || exit 0

source /etc/os-release
[[ ${VERSION_CODENAME:-} == trixie && $(dpkg --print-architecture) == arm64 ]] || {
    echo 'Expected Raspberry Pi OS Lite Trixie ARM64.' >&2; exit 1;
}
[[ -f /boot/firmware/config.txt ]] || { echo 'Missing Pi firmware configuration' >&2; exit 1; }

case "$phase" in
    1)
        export DEBIAN_FRONTEND=noninteractive
        # sdm runs apt update before calling this script inside the image.
        packages=(network-manager dnsmasq-base nftables openssh-server avahi-daemon sudo i2c-tools
            xserver-xorg lightdm openbox pcmanfm lxterminal dbus-x11
            x11-xserver-utils xserver-xorg-input-libinput
            adwaita-icon-theme fonts-dejavu-core
            cmake cmake-curses-gui build-essential pkg-config libgtkmm-3.0-dev)
        apt-get install -y --no-install-recommends "${packages[@]}"
        if ! id "$ROBOT_USER" >/dev/null 2>&1; then useradd -m -s /bin/bash "$ROBOT_USER"; fi
        exit 0
        ;;
    post-install) ;;
esac

install -d -m 755 /etc/miam /etc/NetworkManager/conf.d \
    /etc/lightdm/lightdm.conf.d /etc/X11/xorg.conf.d /usr/share/xsessions \
    /etc/udev/rules.d /etc/modules-load.d /etc/ssh/sshd_config.d
install -d -m 700 /etc/NetworkManager/system-connections
printf '%s:%s\n' "$ROBOT_USER" "$ROBOT_PASSWORD" | chpasswd
# Existing image accounts may have nologin as their shell.
usermod --shell /bin/bash -aG dialout,video,input,render,gpio,i2c,spi,sudo "$ROBOT_USER"
robot_home=$(getent passwd "$ROBOT_USER" | cut -d: -f6)
robot_group=$(id -gn "$ROBOT_USER")
cat > /etc/ssh/sshd_config.d/00-miam.conf <<'EOF'
PasswordAuthentication yes
KbdInteractiveAuthentication no
PermitRootLogin no
EOF
# sdm generates unique host keys during first boot.
rm -f /etc/ssh/ssh_host_*_key /etc/ssh/ssh_host_*_key.pub
printf '%s\n' "$ROBOT_HOSTNAME" > /etc/hostname
sed -i '/^127\.0\.1\.1[[:space:]]/d' /etc/hosts
printf '127.0.1.1\t%s\n' "$ROBOT_HOSTNAME" >> /etc/hosts
# Empty machine-id means systemd generates a unique one on each newly flashed Pi.
truncate -s 0 /etc/machine-id
rm -f /var/lib/dbus/machine-id
ln -s /etc/machine-id /var/lib/dbus/machine-id
rm -f /var/lib/systemd/random-seed

# Keep stock KMS/camera settings. A managed block makes reconfiguration explicit.
sed -i '/^# BEGIN MIAM$/,/^# END MIAM$/d' /boot/firmware/config.txt
cat >> /boot/firmware/config.txt <<EOF

# BEGIN MIAM
[all]
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
dtparam=spi=on
enable_uart=1
dtoverlay=disable-bt
# END MIAM
EOF
systemctl disable hciuart.service 2>/dev/null || true
echo 'i2c-dev' > /etc/modules-load.d/miam-i2c.conf
# Release UART from the serial console, preserving cmdline as a single line.
sed -i -E 's/(^| )console=(serial0|ttyAMA0|ttyS0),[^ ]+//g' /boot/firmware/cmdline.txt
systemctl mask serial-getty@serial0.service serial-getty@ttyAMA0.service serial-getty@ttyS0.service

# NetworkManager owns both interfaces. Do not install standalone dnsmasq,
# hostapd or dhcpcd; dnsmasq-base supplies NM's shared-mode DHCP/DNS process.
cat > /etc/NetworkManager/conf.d/10-miam.conf <<'EOF'
[main]
no-auto-default=*
EOF
connections=/etc/NetworkManager/system-connections
nmcli --offline connection add type wifi ifname wlan0 con-name miam-hotspot \
    autoconnect yes connection.autoconnect-priority 100 \
    ssid "$HOTSPOT_SSID" 802-11-wireless.mode ap 802-11-wireless.band bg \
    802-11-wireless.channel "$HOTSPOT_CHANNEL" 802-11-wireless.powersave 2 \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.proto rsn 802-11-wireless-security.pairwise ccmp \
    802-11-wireless-security.group ccmp 802-11-wireless-security.psk "$HOTSPOT_PASSWORD" \
    ipv4.method shared ipv4.addresses "$HOTSPOT_ADDRESS" \
    ipv4.shared-dhcp-range "$HOTSPOT_DHCP_START,$HOTSPOT_DHCP_END" \
    ipv4.shared-dhcp-lease-time 86400 ipv4.never-default yes ipv6.method disabled \
    > "$connections/miam-hotspot.nmconnection"
nmcli --offline connection add type ethernet ifname eth0 con-name miam-ethernet-dhcp \
    autoconnect yes connection.autoconnect-retries 0 \
    ipv4.method auto ipv4.may-fail no ipv6.method disabled \
    > "$connections/miam-ethernet-dhcp.nmconnection"
chmod 600 "$connections"/miam-*.nmconnection

# Keep hotspot DHCP/DNS, but prevent the Pi from routing traffic between hosts.
# Input/output traffic (SSH, file transfers and the Pi's own internet access)
# is unaffected. Use a separate table without flushing NetworkManager's rules.
cat > /etc/nftables.conf <<'EOF'
#!/usr/sbin/nft -f
table inet miam {
    chain forward {
        type filter hook forward priority 10; policy drop;
    }
}
EOF
chmod 644 /etc/nftables.conf

cat > /etc/udev/rules.d/99-miam-lidar.rules <<'EOF'
# Add ATTRS{serial}=="..." if multiple CP210x serial adapters are attached.
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0660", SYMLINK+="RPLIDAR"
EOF
# The GUI runs as the robot user; allow its existing setpriority(-20) call.
cat > /etc/security/limits.d/90-miam.conf <<EOF
$ROBOT_USER - nice -20
EOF

cat > /usr/share/xsessions/miam.desktop <<'EOF'
[Desktop Entry]
Name=MiAM
Type=Application
Exec=/usr/local/bin/miam-session
EOF
chmod 644 /usr/share/xsessions/miam.desktop
cat > /etc/lightdm/lightdm.conf.d/90-miam.conf <<EOF
[Seat:*]
autologin-user=$ROBOT_USER
autologin-user-timeout=0
autologin-session=miam
user-session=miam
xserver-command=X -s 0 -dpms -nolisten tcp
EOF
chmod 644 /etc/lightdm/lightdm.conf.d/90-miam.conf
printf 'SCREEN_OUTPUT=%q\nSCREEN_ROTATION=%q\n' "$SCREEN_OUTPUT" "$SCREEN_ROTATION" > /etc/miam/display.conf
chmod 644 /etc/miam/display.conf
case "$SCREEN_ROTATION" in
    normal) matrix='1 0 0 0 1 0 0 0 1' ;;
    inverted) matrix='-1 0 1 0 -1 1 0 0 1' ;;
    left) matrix='0 -1 1 1 0 0 0 0 1' ;;
    right) matrix='0 1 0 -1 0 1 0 0 1' ;;
esac
if [[ -z $SCREEN_OUTPUT ]]; then matrix='1 0 0 0 1 0 0 0 1'; fi
cat > /etc/X11/xorg.conf.d/90-miam-touch.conf <<EOF
Section "InputClass"
    Identifier "MiAM touchscreen rotation"
    MatchIsTouchscreen "on"
    Option "TransformationMatrix" "$matrix"
EndSection
EOF
chmod 644 /etc/X11/xorg.conf.d/90-miam-touch.conf
cat > /usr/local/bin/miam-session <<'EOF'
#!/bin/bash
source /etc/miam/display.conf
xset s off
xset s noblank
xset -dpms
if [[ -n $SCREEN_OUTPUT ]]; then
    xrandr --output "$SCREEN_OUTPUT" --rotate "$SCREEN_ROTATION" || logger -t miam-session 'Display rotation failed; inspect xrandr output'
fi
pcmanfm --desktop --profile=miam &
exec openbox-session
EOF
chmod 755 /usr/local/bin/miam-session
install -d -m 755 -o "$ROBOT_USER" -g "$robot_group" \
    "$robot_home/.config" "$robot_home/.config/pcmanfm" "$robot_home/.config/pcmanfm/miam"
cat > "$robot_home/.config/pcmanfm/miam/desktop-items-0.conf" <<'EOF'
[*]
wallpaper_mode=color
desktop_bg=#ffffff
desktop_fg=#000000
desktop_shadow=#ffffff
EOF
chmod 644 "$robot_home/.config/pcmanfm/miam/desktop-items-0.conf"
chown "$ROBOT_USER:$robot_group" "$robot_home/.config/pcmanfm/miam/desktop-items-0.conf"
install -d -m 755 /etc/xdg/libfm
config=/etc/xdg/libfm/libfm.conf
input=$config
[[ -f $config ]] || input=/dev/null
# Preserve other settings; set single_click only in [config].
awk '
    /^\[/ {
        if (in_config)
            print "single_click=1"
        in_config = ($0 == "[config]")
        if (in_config)
            found_config = 1
    }
    in_config && /^[[:space:]]*single_click[[:space:]]*=/ {
        next
    }
    { print }
    END {
        if (!found_config)
            print "[config]"
        if (in_config || !found_config)
            print "single_click=1"
    }
' "$input" > "$config.tmp"
mv -- "$config.tmp" "$config"
chmod 644 "$config"
install -d -m 755 -o "$ROBOT_USER" -g "$robot_group" "$robot_home/Desktop"
cat > "$robot_home/Desktop/match.desktop" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=MatchCode
Comment=Robot match code
Path=$robot_home/
Exec=matchCode
Icon=$robot_home/logo.png
Terminal=false
EOF
chmod 755 "$robot_home/Desktop/match.desktop"
chown "$ROBOT_USER:$robot_group" "$robot_home/Desktop/match.desktop"

# Trimming the SD card stalls disk access and delays robot startup.
# Manual maintenance remains available with sudo fstrim -av.
systemctl disable fstrim.timer
systemctl mask fstrim.service

systemctl unmask avahi-daemon.service avahi-daemon.socket
systemctl enable nftables.service NetworkManager.service ssh.service lightdm.service avahi-daemon.service
systemctl set-default graphical.target
runoneplugin disables post-install 'piwiz|cloudinit'
# The image only needs the password in /etc/shadow, not a second plaintext copy.
rm -f "$assets/robot.conf" "$assets/secrets.conf"
echo 'Robot configuration complete.'
