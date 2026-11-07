# Build the MiAM Raspberry Pi image

Run these commands from this folder on a Linux PC with sudo access and internet
access. Use current filesystem tools compatible with the downloaded image;
Ubuntu 22.04's stock `resize2fs` failed to resize the Trixie image in our tests.

## 1. Download Raspberry Pi OS

Download **Raspberry Pi OS Lite, Trixie, 64-bit** from the
[official archive](https://downloads.raspberrypi.com/raspios_lite_arm64/images/).
Check its published SHA-256 checksum and keep it compressed. The builder accepts
`.img.xz` and `.img.gz`.

## 2. Install sdm

Install [sdm V16.0](https://github.com/gitbls/sdm):

```bash
curl -fsSL https://raw.githubusercontent.com/gitbls/sdm/V16.0/install-sdm -o /tmp/install-sdm
bash /tmp/install-sdm V16.0
```

## 3. Configure and build

`robot.conf` contains the account name, hostname, localization, network and
display settings. Give robots on the same network different hostnames.

Create your credentials file:

```bash
cp secrets.conf.example secrets.conf
chmod 600 secrets.conf
```

Set `ROBOT_PASSWORD`, `HOTSPOT_SSID` and `HOTSPOT_PASSWORD`. SSH login uses
`ROBOT_PASSWORD`. `secrets.conf` is ignored by Git.

Both configuration files are executed as Bash, including as root on the build
PC. Use only files you trust and keep values quoted. The hotspot password must
contain 8–63 printable ASCII characters; the account password must contain no
colon or newline.

Build from the original download:

```bash
./build-image.sh secrets.conf /absolute/path/robot.img.xz
```

The script requests sudo before decompression, runs sdm and creates
`/absolute/path/robot_miam.img.gz`. It keeps the download and removes its temporary
working directory on exit. The image directory must be writable and have no
spaces in its path. Allow space for the expanded image and the compressed result.

An existing output is never overwritten. Move the previous result before
rebuilding. If customization reports an error, resolve it and rebuild from the
original download.

Keep the credentials file and customized images private. The image contains the
hotspot password and account password hash. Deleting the staged credentials file
does not securely erase its contents from the raw image.

## 4. Flash and connect

Use `lsblk` to identify the whole SD card device, for example `/dev/sdX` or
`/dev/mmcblk0`. Unmount its mounted partitions first. This command overwrites the
selected device:

```bash
./flash-image.sh /absolute/path/robot_miam.img.gz /dev/sdX
```

The script requests sudo, unpacks the image and flashes it with root partition
expansion to fill the card. It keeps both files. For subsequent flashes, pass the
unpacked `.img` directly to repeat the same build. If you pass an archive whose
unpacked `.img` already exists, choose whether to flash the existing image,
overwrite it from the archive and flash, or cancel. Cancel is the default; choose
overwrite when the archive contains a newer build.

Boot the Pi and wait for its first-boot setup and automatic restart. sdm generates
unique SSH host keys during this setup, including when the Pi is offline.

Connect to the configured Wi-Fi hotspot:

```bash
ssh pi@192.168.6.2
```

For Ethernet, enable **Shared to other computers** on the PC's Ethernet
connection, connect the Pi and use:

```bash
ssh pi@miam-robot.local
```

These examples use the defaults from `robot.conf`. The PC needs mDNS support to
resolve `.local` names. Reflashing creates new SSH host keys; verify the robot's
identity before replacing a saved host key on the PC.

## Image configuration

- I²C at 400 kHz, SPI and UART enabled; Bluetooth and serial login disabled.
- Hotspot at `192.168.6.2`, with DHCP addresses `.3` through `.20` and one-day leases.
- Ethernet DHCP for address, gateway and DNS; Avahi advertises the hostname.
- IPv6 disabled on the two configured network profiles.
- nftables blocks routed traffic through the Pi. SSH, file transfers, hotspot
  DHCP and the Pi's own internet access remain available.
- Password SSH login; direct root SSH login disabled.
- LightDM, Openbox, PCManFM and LXTerminal; automatic desktop login, white
  background and single-click icons.
- Display and touch rotated 180° on `DSI-1`; screen blanking disabled. Change
  `SCREEN_OUTPUT` and `SCREEN_ROTATION` for another display. Touch rotation applies
  to all touchscreens and assumes a single-screen setup.
- GTK/gtkmm runtime and development headers, GCC/G++, Make, pkg-config, CMake and
  `ccmake`.
- Robot account in the sudo and hardware-access groups, with permission to use
  nice priority -20.
- `/dev/RPLIDAR` for serial devices matching USB vendor/product `10c4:ea60`.

The desktop icon runs `matchCode` from the user's home directory and uses
`~/logo.png`. Install `matchCode` on the desktop session's `PATH` and place the
logo in the home directory.

Automatic filesystem trimming is disabled to avoid the observed SD-card boot
delay. Run it manually while the robot is idle:

```bash
sudo fstrim -av
```

## How customization works

`build-image.sh` handles decompression, sdm and compression. sdm calls
`configure-image.sh` in three phases: copy configuration into the mounted image,
install packages inside it, then write the final system configuration. sdm also
updates packages and arranges localization, desktop boot behavior and the
first-boot restart. Package versions follow the configured repositories.

The customization disables the Pi first-run wizard and user-configuration
service because the robot account is already configured. If cloud-init is
installed, sdm disables it and puts the `cloud-init` package on hold, preventing
normal package upgrades from updating it.

The persistent custom session is `/usr/local/bin/miam-session`. It applies screen
settings, starts desktop icons and runs Openbox at each graphical login. Network
profiles are in `/etc/NetworkManager/system-connections/`; the forwarding rule is
in `/etc/nftables.conf`. Check that rule on the Pi with
`sudo nft list table inet miam`: the `forward` chain should have `policy drop`.
