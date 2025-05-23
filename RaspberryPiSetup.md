# Image creation

## Initial setup

 - Use raspberry pi imager to flash Raspi OS Lite. On edit settings, create user account + enable ssh
 - Modify `config.txt` to include the following:

```
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
```

 - Boot, connect to the internet and run the following

```
sudo apt update && sudo apt upgrade -y
sudo apt install hostapd dhcpcd dnsmasq i2c-tools
sudo systemctl unmask hostapd && sudo systemctl enable hostapd
```

 - Run `raspi-config` and enable spi, camera...



### Configure WiFi network broadcast

 - Create WiFi hotspot: create a file called `/etc/hostadp/hostapd.conf`

```
country_code=FR
interface=wlan0
ssid=<SSID>
hw_mode=g
channel=7
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=<password>
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
wpa_group_rekey=86400
```

 - Create DNS server: append to `/etc/dnsmasq.conf`

```
interface=wlan0
	dhcp-range=192.168.6.3,192.168.6.20,255.255.255.0,24h
```

 - Give RPi an address in WiFi, and ethernet fallback: append to `/etc/dhcpcd.conf`

```
profile static_eth0
static ip_address=172.16.0.142/21
static routers=172.16.0.1
static domain_name_servers=172.16.0.1 8.8.8.8

# fallback to static profile on eth0
interface eth0
fallback static_eth0

interface wlan0
    static ip_address=192.168.6.2/24
    nohook wpa_supplicant
```

 - Edit the file `/etc/systemd/system/multi-user.target.wants/hostapd.service` to add a small delay:
 `ExecStartPre=/bin/sleep 1`

 - Use `raspi-config` Localization options to set WLAN country

# Minimal desktop environment to run GTK app

This follows the tutorial found at: https://forums.raspberrypi.com/viewtopic.php?f=66&t=133691#p890408

This install Xorg and RPD stripped.

```
sudo apt-get install --no-install-recommends xserver-xorg xinit
sudo apt-get install --no-install-recommends raspberrypi-ui-mods lxsession
```


## Rotate screen

Add to `/boot/config.txt`:

```
display_lcd_rotate=2
lcd_rotate=2
dtoverlay=disable-bt,
```

First line is for screen display, second for touchscreen coordinates ; 2 = 180deg

## Prevent screensaver

Add the following files:
/etc/X11/xorg.conf.d/10-extensions.conf
```
Section "Extensions"
    Option "DPMS" "false"
EndSection
```
/etc/X11/xorg.conf.d/10-serverflags.conf
```
Section "ServerFlags"
    Option "BlankTime" "0"
EndSection
```


To check: `DISPLAY=:0 xset q` should show

`
Screen Saver:
  prefer blanking:  yes    allow exposures:  yes
  timeout:  0    cycle:  0

DPMS (Energy Star):
  Standby: 600    Suspend: 600    Off: 600
  DPMS is Disabled

`

## Enable automatic login

Use `raspi-config`, go to System Options /  Boot / Auto Login and select the corresponding option

# Create disk image

## Shrink partition

Mount the SD card to a PC, then shrink following instructions from https://access.redhat.com/articles/1196333


First, shrink the filesystem as much as possible:

```
sudo e4defrag /dev/sda2

sudo umount /dev/sda2
sudo e2fsck -f /dev/sda2
sudo resize2fs /dev/sda2 -M
```

Then shrink the partition - using disk GUI or manually:


```
sudo fdisk /dev/sda
  p -> find line_number
  d line_number
  n
    p
    Set start sector to be the same as before
    Set end sector to (blockcount + 1) * blocksize
  w
```

## Create image

sudo dd bs=512 if=/dev/sda status=progress count=<fdisk -l =\> end + 1> | gzip --fast > output.gz

## Apply image

`
sudo gzip -dc output.gz | dd bs=512 of=/dev/sdb
`

Use `raspi-config` to resize partititon to fill full SD card.

## Sharing internet over Wifi

 - On raspberry: `sudo route add default gw 172.16.0.1`
 - On PC with IP `172.16.0.1`

```
echo 1 > /proc/sys/net/ipv4/ip_forward
iptables -A FORWARD -i enp2s0 -o wlp0s20f3 -j ACCEPT
iptables -A FORWARD -i wlp0s20f3 -o enp2s0 -j ACCEPT
iptables -t nat -A POSTROUTING -o wlp0s20f3 -j MASQUERADE
```

## Launch match code

Generate an executable file `~Desktop/match.desktop`

```
[Desktop Entry]
Type=Application
Version=1.0
Name=MatchCode
Comment=Robot match code
Path=/home/pi/
Exec=matchCode
Icon=/home/pi/logo.png
Terminal=false
```

## udev rules

To be placed in `/etc/udev/rules.d`

Lidar:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="RPLIDAR"
```


## Single click destop icon

Modify `/etc/xdg/libfm/libfm.conf` to set `single_click=1`