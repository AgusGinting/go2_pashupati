#!/usr/bin/env bash
# The Go2 robot package's installer (ROBOT_SETUP in /etc/pashupati/pashupati.env), run by
# `scripts/setup.sh check` and `scripts/setup.sh install robot`:
#
#   setup.sh check     # unitree_sdk2py importable, the robot answers on its interface
#   setup.sh install   # cyclonedds 0.10.2 and unitree_sdk2py (pinned) for this user
#
# Env: GO2_IFACE (eth0), ROBOT_IP (192.168.123.161), UNITREE_SDK_REF (the pinned commit),
#      CYCLONEDDS_HOME (a Cyclone DDS 0.10 C build; built in ~/cyclonedds/install when pip
#      finds no wheel, e.g. on aarch64), FORCE=1 (reinstall anyway)
set -euo pipefail

GO2_IFACE="${GO2_IFACE:-eth0}"
ROBOT_IP="${ROBOT_IP:-192.168.123.161}"
# the commit on the field-tested Go2 EDU (2025-07-09)
UNITREE_SDK_REF="${UNITREE_SDK_REF:-f5592911758bf20c4b2b3124c9421d89c685325d}"
UNITREE_SDK_URL="https://github.com/unitreerobotics/unitree_sdk2_python"
CYCLONEDDS_VERSION=0.10.2
[[ -n "${CYCLONEDDS_HOME:-}" || ! -d "$HOME/cyclonedds/install" ]] ||
    export CYCLONEDDS_HOME="$HOME/cyclonedds/install"

# as the driver runs: the SDK's libddsc before a sourced ROS's (launch/bringup.launch.py)
sdk_ok() {
    LD_LIBRARY_PATH="${CYCLONEDDS_HOME:+$CYCLONEDDS_HOME/lib:}${LD_LIBRARY_PATH:-}" \
        python3 -c 'import cyclonedds, unitree_sdk2py.go2.sport.sport_client' 2>/dev/null
}

check() {
    local ok=0
    if sdk_ok; then
        echo "unitree_sdk2py: ok"
    else
        echo "unitree_sdk2py: missing (setup.sh install)"
        ok=1
    fi
    if ip link show "$GO2_IFACE" >/dev/null 2>&1; then
        if ping -c1 -W1 -I "$GO2_IFACE" "$ROBOT_IP" >/dev/null 2>&1; then
            echo "robot $ROBOT_IP on $GO2_IFACE: ok"
        else
            echo "robot $ROBOT_IP on $GO2_IFACE: no answer"
            ok=1
        fi
    else
        echo "interface $GO2_IFACE: missing (GO2_IFACE, network_interface in go2_driver.yaml)"
        ok=1
    fi
    return $ok
}

# Cyclone DDS 0.10.x (C) for pip to build the Python binding against
build_cyclonedds() {
    local src="$HOME/cyclonedds"
    [[ -d "$src/.git" ]] || git clone --depth 1 -b releases/0.10.x \
        https://github.com/eclipse-cyclonedds/cyclonedds "$src"
    cmake -S "$src" -B "$src/build" -DCMAKE_INSTALL_PREFIX="$src/install" -DBUILD_EXAMPLES=OFF
    cmake --build "$src/build" --target install -j"$(nproc)"
    export CYCLONEDDS_HOME="$src/install"
}

install() {
    if sdk_ok && [[ "${FORCE:-0}" != 1 ]]; then
        echo "unitree_sdk2py already installed (FORCE=1 to reinstall)"
        return 0
    fi
    python3 -m pip install --user "cyclonedds==$CYCLONEDDS_VERSION" || {
        [[ -n "${CYCLONEDDS_HOME:-}" ]] || build_cyclonedds
        python3 -m pip install --user "cyclonedds==$CYCLONEDDS_VERSION"
    }
    # --no-deps: its opencv-python would shadow ROS's cv2 and its numpy; ROS has numpy
    python3 -m pip install --user --no-deps "unitree_sdk2py @ git+$UNITREE_SDK_URL@$UNITREE_SDK_REF"
    sdk_ok && echo "unitree_sdk2py: ok"
}

case "${1:-check}" in
    check) check ;;
    install) install ;;
    *)
        echo "usage: $0 check|install" >&2
        exit 2
        ;;
esac
