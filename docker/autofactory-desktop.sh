#!/usr/bin/env bash
# Dev-only display setup. Export the same settings into every terminal.
set -euo pipefail
mkdir -p /run/autofactory /tmp/autofactory-desktop
chmod 755 /tmp/autofactory-desktop
# The container entrypoint stays root; VS Code and applications run as ros.
# Copy the host cookie rather than changing ownership of a bind-mounted file.
if [[ -n ${XAUTHORITY:-} && -f $XAUTHORITY ]]; then
    install -m 600 -o ros -g "$(id -gn ros)" "$XAUTHORITY" /run/autofactory/Xauthority
    export XAUTHORITY=/run/autofactory/Xauthority
fi
# Device group IDs vary across hosts. Grant ros access without chmod on devices.
for device in /dev/dri/* /dev/dxg /dev/nvidia*; do
    [[ -c $device ]] || continue
    device_gid=$(stat -c %g "$device")
    [[ $device_gid != 0 ]] || continue
    if ! getent group "$device_gid" >/dev/null; then
        groupadd --gid "$device_gid" "host-device-$device_gid"
    fi
    usermod -aG "$(getent group "$device_gid" | cut -d: -f1)" ros
done
graphics_probe() {
    sudo -H -u ros --preserve-env=DISPLAY,XAUTHORITY,LD_LIBRARY_PATH,GALLIUM_DRIVER,LIBGL_ALWAYS_SOFTWARE \
        timeout 15 glxinfo -B
}
browser() {
    export DISPLAY=:99
    unset XAUTHORITY GALLIUM_DRIVER
    export LIBGL_ALWAYS_SOFTWARE=1
    Xvfb :99 -screen 0 1600x900x24 -nolisten tcp > /tmp/autofactory-desktop/xvfb.log 2>&1 &
    for _ in {1..50}; do
        if xdpyinfo -display :99 >/dev/null 2>&1; then break; fi
        sleep 0.1
    done
    xdpyinfo -display :99 >/dev/null 2>&1 || { echo 'Virtual display failed'; exit 1; }
    openbox > /tmp/autofactory-desktop/openbox.log 2>&1 &
    x11vnc -display :99 -localhost -rfbport 5900 -forever -shared -nopw \
        > /tmp/autofactory-desktop/vnc.log 2>&1 &
    websockify --web=/usr/share/novnc 0.0.0.0:6080 localhost:5900 \
        > /tmp/autofactory-desktop/websockify.log 2>&1 &
    python3 - <<'PYREADY'
import time
import urllib.request
for attempt in range(50):
    try:
        with urllib.request.urlopen('http://127.0.0.1:6080/vnc.html', timeout=1) as response:
            assert response.status == 200
        break
    except Exception:
        time.sleep(0.1)
else:
    raise SystemExit('Browser desktop failed; see /tmp/autofactory-desktop/*.log')
PYREADY
    echo 'Desktop: http://localhost:6080/vnc.html?autoconnect=1&resize=scale'
}
if [[ ${AUTOFACTORY_DISPLAY_MODE:-browser} == native ]] && timeout 5 xdpyinfo >/dev/null 2>&1; then
    if [[ ${AUTOFACTORY_GPU:-auto} != cpu ]] && graphics_probe > /run/autofactory/glxinfo.log 2>&1; then
        if grep -Eiq 'llvmpipe|softpipe|software rasterizer' /run/autofactory/glxinfo.log; then
            unset GALLIUM_DRIVER
            export LIBGL_ALWAYS_SOFTWARE=1
        else
            unset LIBGL_ALWAYS_SOFTWARE
        fi
    else
        unset GALLIUM_DRIVER
        export LIBGL_ALWAYS_SOFTWARE=1
    fi
    # A usable X server can still lack the GL context Gazebo needs.
    if ! graphics_probe >/run/autofactory/glxinfo.log 2>&1; then browser; fi
else
    browser
fi
# Ogre-Next enumerates EGL devices and picks the DRM node even when software rendering is
# forced, which makes Mesa segfault. Hide the nodes whenever rendering is on the CPU.
if [[ ${LIBGL_ALWAYS_SOFTWARE:-0} == 1 && -d /dev/dri ]]; then
    rm -rf /dev/dri
fi
{
    printf 'export DISPLAY=%q\n' "$DISPLAY"
    for key in LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER XAUTHORITY LD_LIBRARY_PATH; do
        if [[ -v $key ]]; then printf 'export %s=%q\n' "$key" "${!key}"; else printf 'unset %s\n' "$key"; fi
    done
} > /run/autofactory/rendering.sh
chmod 644 /run/autofactory/rendering.sh
printf 'Display=%s; software rendering=%s\n' "$DISPLAY" "${LIBGL_ALWAYS_SOFTWARE:-0}"
exec /ros_entrypoint.sh "$@"
