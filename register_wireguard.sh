#!/bin/bash
# Register this dev PC's WireGuard public key on whichever robot is currently
# connected (192.168.37.10). Run once per robot after devpc_setup.sh.
set -e

ROBOT_IP="192.168.37.10"
ROBOT_USER="test"
ROBOT_PASS="123456"

if ! command -v sshpass &>/dev/null; then
    echo "ERROR: sshpass not installed. Run: sudo apt install sshpass"
    exit 1
fi

echo -n "Reading dev PC WireGuard public key ... "
if ip link show wg0 &>/dev/null 2>&1; then
    DEV_PUB=$(sudo wg show wg0 public-key)
elif [ -f /etc/wireguard/wg0.conf ]; then
    DEV_PUB=$(sudo awk '/PrivateKey/ {print $3}' /etc/wireguard/wg0.conf | wg pubkey)
else
    echo "FAILED"
    echo "ERROR: WireGuard not configured. Run devpc_setup.sh first."
    exit 1
fi
echo "$DEV_PUB"

echo -n "Connecting to robot at ${ROBOT_IP} ... "
if ! ping -c 1 -W 2 "$ROBOT_IP" &>/dev/null; then
    echo "FAILED"
    echo "ERROR: Robot not reachable. Make sure you are connected to its network."
    exit 1
fi
echo "OK"

echo -n "Updating WireGuard peer on robot ... "
OLD_PUB=$(sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
    "${ROBOT_USER}@${ROBOT_IP}" \
    "echo '${ROBOT_PASS}' | sudo -kS wg show wg0 allowed-ips 2>/dev/null \
     | grep '10\.0\.0\.1/32' | awk '{print \$1}'" 2>/dev/null || true)

if [ -n "$OLD_PUB" ] && [ "$OLD_PUB" = "$DEV_PUB" ]; then
    echo "already registered."
    exit 0
fi

if [ -n "$OLD_PUB" ]; then
    sshpass -p "$ROBOT_PASS" ssh "${ROBOT_USER}@${ROBOT_IP}" \
        "echo '${ROBOT_PASS}' | sudo -kS wg set wg0 peer ${OLD_PUB} remove" 2>/dev/null || true
fi

sshpass -p "$ROBOT_PASS" ssh "${ROBOT_USER}@${ROBOT_IP}" \
    "echo '${ROBOT_PASS}' | sudo -kS wg set wg0 peer ${DEV_PUB} \
     allowed-ips 10.0.0.1/32 persistent-keepalive 25 && \
     echo '${ROBOT_PASS}' | sudo -kS wg-quick save wg0"
echo "Done."
