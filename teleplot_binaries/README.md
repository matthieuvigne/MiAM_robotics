# Binaries for teleplot

 - `teleplot-linux`: for linux x86, used by the docker in simulation
 - `teleplot-armv7`: a cross-compiled version for Rasbperry Pi V3. *Note: requires Glibc >=2.29*

## Deploy on Raspberry pi

 - Copy binary `teleplot-armv7` in `/home/pi`
 - Create a file `/etc/systemd/system/TeleplotService.service` with the following
```
[Unit]
Description=Start teleplot server
After=multi-user.target
[Service]
Type=simple
ExecStart=/home/pi/teleplot-armv7
User=pi
[Install]
WantedBy=multi-user.target
```

 - Load service: `sudo systemctl daemon-reload && sudo systemctl enable TeleplotService`

 - Reboot. The service is now launched at boot.
   You can now connect to the telemetry window from your PC at <raspberry pi IP>:8080 (typically http://192.168.6.2:8080/)
