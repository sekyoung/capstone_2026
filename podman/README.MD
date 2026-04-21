# Podman — capstone2026

## Build

```bash
podman build -t capstone2026 .
```

---

## Run

### Default (`rotating_test.py`)

```bash
podman run --rm -it \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  capstone2026
```

### WASD Teleop

```bash
podman run --rm -it \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  capstone2026 --launch wasd_test.py
```

### List Available Launch Files

```bash
podman run --rm capstone2026 --list
```

---

## Troubleshooting (Fedora — SELinux & Wayland)

### SELinux denial for RViz2

```bash
# 1. Install SELinux policy tools
sudo dnf install -y policycoreutils-devel selinux-policy-devel

# 2. Generate a policy module from the denial log
sudo ausearch -m avc -ts recent | audit2allow -M rviz2_x11

# 3. Install the generated module
sudo semodule -i rviz2_x11.pp

# Verify
sudo semodule -l | grep rviz2
```

### Black Screen

First, grant X11 access:

```bash
xhost +local:
```

**Software rendering (no GPU):**

```bash
podman run --rm -it \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -e QT_QPA_PLATFORM=xcb \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e MESA_GL_VERSION_OVERRIDE=3.3 \
  capstone2026
```

**Hardware rendering (Intel/AMD GPU):**

Check your device path first: `ls /dev/dri/`

```bash
podman run --rm -it \
  --network=host \
  --device /dev/dri \
  -e DISPLAY=$DISPLAY \
  -e QT_QPA_PLATFORM=xcb \
  capstone2026
```

> **Cleanup:** When done, revoke X11 access and (if applicable) remove the SELinux module:
> ```bash
> xhost -local:
> sudo semodule -r rviz2_x11   # only if you have strict security concerns
> ```
