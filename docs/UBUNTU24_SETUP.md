# Running topstar_ros2 on Ubuntu 24.04

topstar_ros2 requires Ubuntu 22.04 / ROS 2 Humble / Python 3.10 (the H1 hardware
backend's `xapi` vendor wheel is built for `cp310`). On Ubuntu 24.04 the cleanest
solution is **Distrobox** — a thin wrapper around Podman that creates a fully
integrated Ubuntu 22.04 container while sharing your home directory, display, and
network transparently.

---

## 1. Install Distrobox and Podman

```bash
sudo apt install -y distrobox podman
```

---

## 2. Pull the Ubuntu 22.04 image

Docker Hub is reachable from most networks. If it is blocked, use the DaoCloud
mirror:

```bash
# Standard
podman pull ubuntu:22.04

# If Docker Hub is blocked (e.g. mainland China)
podman pull docker.m.daocloud.io/library/ubuntu:22.04
podman tag docker.m.daocloud.io/library/ubuntu:22.04 docker.io/library/ubuntu:22.04
```

---

## 3. Create the container

The bare Ubuntu 22.04 image has no `ca-certificates`, so HTTPS apt sources fail
on first entry. The workaround is to mount the host's CA bundle into the container
at a temporary path and copy it before `apt` runs, while also switching apt to use
HTTPS sources (needed if a transparent HTTP proxy is present on the network).

```bash
distrobox create --name ros2-humble --image ubuntu:22.04 --yes \
  --volume /etc/ssl/certs/ca-certificates.crt:/tmp/host-ca.crt:ro \
  --pre-init-hooks "
    mkdir -p /etc/ssl/certs
    cp /tmp/host-ca.crt /etc/ssl/certs/ca-certificates.crt
    sed -i 's|http://archive.ubuntu.com/ubuntu|https://archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list
    sed -i 's|http://security.ubuntu.com/ubuntu|https://security.ubuntu.com/ubuntu|g' /etc/apt/sources.list
  "
```

> **Why not mount directly to `/etc/ssl/certs/ca-certificates.crt`?**
> The `ca-certificates` post-install script replaces that file with `mv`. A
> bind-mount makes the destination read-only at the kernel level, causing
> `mv: Device or resource busy`. Mounting to `/tmp/host-ca.crt` and copying it
> leaves the real path writable.

Verify the container initialises cleanly:

```bash
distrobox enter ros2-humble -- bash -c 'cat /etc/os-release | grep PRETTY'
# PRETTY_NAME="Ubuntu 22.04.x LTS"
```

---

## 4. Install ROS 2 Humble

### 4a. Add the ROS 2 GPG key

`raw.githubusercontent.com` may be unreachable from inside the container.
Download the key on the **host** and copy it in via `/run/host/`:

```bash
# On the host
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /tmp/ros.key

# Copy into the running container
podman start ros2-humble
podman exec --user root ros2-humble \
  bash -c 'cp /run/host/tmp/ros.key /usr/share/keyrings/ros-archive-keyring.gpg'
```

### 4b. Run the install script

Create `/tmp/ros2_install.sh`:

```bash
#!/bin/bash
set -e
export DEBIAN_FRONTEND=noninteractive

# Use TUNA mirror for ROS 2 (packages.ros.org may have SSL issues behind some proxies)
ARCH=$(dpkg --print-architecture)
echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt-get update -q
sudo apt-get install -y -q \
  build-essential \
  ros-humble-ros-base \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosidl-generator-dds-idl \
  libyaml-cpp-dev \
  python3-pip \
  python3-colcon-common-extensions

grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc || \
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Then run it:

```bash
distrobox enter ros2-humble -- bash /tmp/ros2_install.sh
```

---

## 5. Fix the ROS environment in `~/.bashrc`

If ROS 2 Jazzy is installed on the Ubuntu 24.04 host (e.g. via fishros), its
environment variables (`ROS_DISTRO`, `AMENT_PREFIX_PATH`, etc.) leak into the
container. Distrobox sets `$CONTAINER_ID` inside the container, so you can
gate each distribution:

```bash
# Replace the unconditional Jazzy source block with:
# >>> fishros initialize >>>
if [ -z "$CONTAINER_ID" ]; then
    source /opt/ros/jazzy/setup.bash
fi
# <<< fishros initialize <<<

# And replace the unconditional Humble source line with:
if [ -n "$CONTAINER_ID" ]; then
    unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION \
          AMENT_PREFIX_PATH CMAKE_PREFIX_PATH \
          COLCON_PREFIX_PATH AMENT_CURRENT_PREFIX
    source /opt/ros/humble/setup.bash
fi
```

---

## 6. Build the workspace

```bash
distrobox enter ros2-humble

# Interface packages (required first)
source /opt/ros/humble/setup.bash
cd ~/topstar_ros2/cyclonedds_ws
colcon build

# H1 example package
source ~/topstar_ros2/setup_local.sh   # loopback (sim) — or setup.sh for real robot
cd ~/topstar_ros2/example
bash build_h1.sh

# H2 example package
source ~/topstar_ros2/setup.sh
cd ~/topstar_ros2/example
colcon build --packages-select topstar_ros2_h2_example
```

---

## 7. PySide6 GUI support

The H1 upper-body jog GUI (`h1_upper_body_jog`) requires PySide6 and several
xcb system libraries that are absent from the minimal Ubuntu 22.04 container.

```bash
# Inside the container
sudo apt-get install -y \
  libxkbcommon-x11-0 \
  libxcb-icccm4 \
  libxcb-image0 \
  libxcb-keysyms1 \
  libxcb-randr0 \
  libxcb-render-util0 \
  libxcb-shape0 \
  libxcb-xinerama0 \
  libxcb-xkb1 \
  libxcb-cursor0

sudo pip3 install PySide6
```

> **Note:** Qt 6.5+ prints a warning about `xcb-cursor0` being needed, but the
> actual blocker is `libxkbcommon-x11.so.0`. Install all libraries above to
> avoid chasing the wrong error.

Distrobox shares the host display automatically. No `DISPLAY` configuration is
needed when entering interactively from a desktop session.

---

## Known limitations

| Area | Notes |
|---|---|
| GPU / MuJoCo viewer | `/dev/dri` not shared by default. Add `--additional-flags "--device /dev/dri"` to `distrobox create` if you need the MuJoCo viewer window. |
| USB / serial (xapi hardware) | Add `--additional-flags "--device /dev/ttyUSB0"` (or the relevant device) at container creation time. |
| `host-spawn: command not found` | Harmless warning on container entry. Only affects launching host GUI apps from inside the container. |
| Systemd | Not running inside the container. No impact on topstar_ros2. |
