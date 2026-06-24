#!/bin/bash
# Register this dev PC's WireGuard public key on whichever robot is currently
# connected (192.168.37.10). Each dev PC gets a unique IP in 10.0.0.10+ so
# multiple dev PCs can be registered simultaneously without replacing each other.
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
    DEV_PUB=$(echo '123456' | sudo -kS wg show wg0 public-key 2>/dev/null)
elif [ -f /etc/wireguard/wg0.conf ]; then
    DEV_PUB=$(echo '123456' | sudo -kS awk '/PrivateKey/ {print $3}' /etc/wireguard/wg0.conf 2>/dev/null | wg pubkey)
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

# Get all current peers from robot: lines of "<pubkey>  <allowedIP/prefix>"
WG_PEERS=$(sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
    "${ROBOT_USER}@${ROBOT_IP}" \
    "echo '${ROBOT_PASS}' | sudo -kS wg show wg0 allowed-ips 2>/dev/null" 2>/dev/null || true)

# Check if this dev PC is already registered (match by public key)
ASSIGNED_IP=$(echo "$WG_PEERS" | grep -F "$DEV_PUB" | awk '{print $2}' | cut -d'/' -f1 || true)

if [ -n "$ASSIGNED_IP" ]; then
    echo "Already registered as ${ASSIGNED_IP} — nothing to do."
else
    # Prefer the IP already in local wg0 (so the same PC gets the same IP on all robots)
    LOCAL_IP=""
    if ip link show wg0 &>/dev/null 2>&1; then
        LOCAL_IP=$(ip -o addr show wg0 | awk '/10\.0\.0\./ {split($4,a,"/"); print a[1]}')
    elif [ -f /etc/wireguard/wg0.conf ]; then
        LOCAL_IP=$(echo '123456' | sudo -kS awk '/^Address/ {split($3,a,"/"); print a[1]}' \
            /etc/wireguard/wg0.conf 2>/dev/null || true)
    fi

    USED_IPS=$(echo "$WG_PEERS" | awk '{print $2}' | cut -d'/' -f1 | grep '^10\.0\.0\.' || true)

    # Use local IP if it's in the dev-PC range and not already taken by another key
    if echo "$LOCAL_IP" | grep -qE '^10\.0\.0\.([1-9][0-9]|[2-9][0-9]{1,2}|1[0-9]{2}|2[0-4][0-9]|25[0-4])$' && \
       ! echo "$USED_IPS" | grep -qF "$LOCAL_IP"; then
        ASSIGNED_IP="$LOCAL_IP"
    else
        # Auto-assign next available from 10.0.0.10
        for i in $(seq 10 254); do
            CANDIDATE="10.0.0.$i"
            if ! echo "$USED_IPS" | grep -qF "$CANDIDATE"; then
                ASSIGNED_IP="$CANDIDATE"
                break
            fi
        done
    fi

    [ -n "$ASSIGNED_IP" ] || { echo "ERROR: No available IPs in 10.0.0.10–254"; exit 1; }

    echo -n "Registering as ${ASSIGNED_IP} ... "
    sshpass -p "$ROBOT_PASS" ssh "${ROBOT_USER}@${ROBOT_IP}" \
        "echo '${ROBOT_PASS}' | sudo -kS wg set wg0 peer ${DEV_PUB} \
         allowed-ips ${ASSIGNED_IP}/32 persistent-keepalive 25 && \
         echo '${ROBOT_PASS}' | sudo -kS wg-quick save wg0"
    echo "Done."
fi

# Add dev PC IP to Computer A's CycloneDDS peer list so the bridge discovers it proactively
echo -n "Updating CycloneDDS peer list on robot ... "
if sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
        "${ROBOT_USER}@${ROBOT_IP}" \
        "grep -qF 'Address=\"${ASSIGNED_IP}\"' /etc/cyclonedds/config.xml" 2>/dev/null; then
    echo "already present."
else
    sshpass -p "$ROBOT_PASS" ssh "${ROBOT_USER}@${ROBOT_IP}" \
        "echo '${ROBOT_PASS}' | sudo -kS sed -i \
         's|</Peers>|        <Peer Address=\"${ASSIGNED_IP}\"/>\n      </Peers>|' \
         /etc/cyclonedds/config.xml && \
         echo '${ROBOT_PASS}' | sudo -kS systemctl restart topstar_bridge_v2.service" \
        && echo "Done (bridge restarted)." || echo "FAILED."
fi

echo "WireGuard IP for this dev PC: ${ASSIGNED_IP}"
