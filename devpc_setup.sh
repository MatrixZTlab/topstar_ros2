#!/bin/bash
# Run this script on a new dev PC after cloning the topstar_ros2 repo.
# It detects your network interfaces, sets up routing to subnet-37 persistently,
# generates all robot setup scripts, and configures WireGuard.
# A and B require no changes — they respond to any incoming DDS peer.
set -e

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== TopStar ROS2 Dev PC Setup ==="
echo ""

# ── 1. Detect interfaces ──────────────────────────────────────────────────────

WIRED_IF=$(ip -o addr show | awk '/192\.168\.36\./ {print $2}' | head -1)
if [ -z "$WIRED_IF" ]; then
    echo "ERROR: No interface with a 192.168.36.x address found."
    echo "       Connect the wired link to subnet-36 and assign a static IP first."
    exit 1
fi
WIRED_IP=$(ip -o addr show "$WIRED_IF" | awk '/192\.168\.36\./ {split($4,a,"/"); print a[1]}')
echo "Wired interface : $WIRED_IF  ($WIRED_IP)"

WIFI_IF=$(ip -o addr show | awk '/192\.168\.1\./ {print $2}' | grep -v '^lo$' | head -1)
if [ -z "$WIFI_IF" ]; then
    echo "WiFi interface  : not found on 192.168.1.x — WiFi setup scripts will be skipped"
else
    WIFI_IP=$(ip -o addr show "$WIFI_IF" | awk '/192\.168\.1\./ {split($4,a,"/"); print a[1]}')
    echo "WiFi interface  : $WIFI_IF  ($WIFI_IP)"
fi

# ── 2. Static route to subnet-37 ─────────────────────────────────────────────

echo ""
echo "=== Static route to 192.168.37.0/24 ==="

if ip route show | grep -q "192\.168\.37\.0/24"; then
    echo "Route already present."
else
    sudo ip route add 192.168.37.0/24 via 192.168.36.10
    echo "Route added (active now)."
fi

CONN=$(nmcli -t -f NAME,DEVICE,STATE connection show --active 2>/dev/null \
       | awk -F: -v iface="$WIRED_IF" '$2==iface && $3=="activated" {print $1; exit}')
if [ -z "$CONN" ]; then
    echo "WARNING: NetworkManager connection for $WIRED_IF not found."
    echo "         Persist the route manually after identifying the connection name:"
    echo "           nmcli -t -f NAME,DEVICE,STATE connection show --active"
    echo "           sudo nmcli connection modify \"<name>\" +ipv4.routes \"192.168.37.0/24 192.168.36.10\""
else
    if nmcli connection show "$CONN" 2>/dev/null | grep -q "192\.168\.37\.0/24"; then
        echo "Persistent route already set on connection \"$CONN\"."
    else
        sudo nmcli connection modify "$CONN" +ipv4.routes "192.168.37.0/24 192.168.36.10"
        sudo nmcli connection up "$CONN" > /dev/null
        echo "Persistent route saved to connection \"$CONN\"."
    fi
fi

echo -n "Connectivity check — ping Computer A (192.168.37.10) ... "
if ping -c 1 -W 2 192.168.37.10 &>/dev/null; then
    echo "OK"
else
    echo "FAILED"
    echo "  Check: is Computer B's IP forwarding on?  (cat /proc/sys/net/ipv4/ip_forward on B)"
    echo "  Check: does Computer A have a return route? (ip route show on A, expect 192.168.36.0/24 via 192.168.37.11)"
fi

# ── 3. Generate wired/WiFi setup scripts ─────────────────────────────────────

echo ""
echo "=== Generating setup scripts in $REPO ==="

write_setup_script() {
    local file="$1"
    local label="$2"
    local domain="$3"
    local uri="$4"
    local uri_comment="$5"
    cat > "$file" << SCRIPT
#!/bin/bash
echo "Setup topstar ros2 environment ($label)"
source /opt/ros/humble/setup.bash
source \$HOME/topstar_ros2/cyclonedds_ws/install/setup.bash
if [ -f "\$HOME/topstar_ros2/example/install/setup.bash" ]; then
    source \$HOME/topstar_ros2/example/install/setup.bash
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=$domain
# $uri_comment
# IMPORTANT: keep CYCLONEDDS_URI on one line — CycloneDDS fails to parse multiline env vars.
export CYCLONEDDS_URI='$uri'
# Stop any stale ros2 daemon (graceful stop cleans socket files; pkill leaves them and causes !rclpy.ok() errors)
ros2 daemon stop 2>/dev/null || true
SCRIPT
    chmod +x "$file"
    echo "  Written: $file"
}

# WiFi URIs — same subnet as A's wlp4s0, only need A's WiFi IP as peer
if [ -n "$WIFI_IF" ]; then
    WIFI_URI_R2="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$WIFI_IF\" priority=\"default\" multicast=\"default\"/></Interfaces><MaxMessageSize>1438B</MaxMessageSize></General><Discovery><Peers><Peer Address=\"192.168.1.12\"/></Peers></Discovery></Domain></CycloneDDS>"
    WIFI_URI_R1="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$WIFI_IF\" priority=\"default\" multicast=\"default\"/></Interfaces><MaxMessageSize>1438B</MaxMessageSize></General><Discovery><Peers><Peer Address=\"192.168.1.11\"/></Peers></Discovery></Domain></CycloneDDS>"

    write_setup_script "$REPO/setup.sh" "Robot 2, domain 2, WiFi" 2 \
        "$WIFI_URI_R2" \
        "WiFi path: A's wlp4s0 is 192.168.1.12 — same subnet, multicast + unicast peer."

    write_setup_script "$REPO/setup_r1.sh" "Robot 1, domain 1, WiFi" 1 \
        "$WIFI_URI_R1" \
        "WiFi path: A's wlp4s0 is 192.168.1.11 — same subnet, multicast + unicast peer."
else
    echo "  Skipped: setup.sh and setup_r1.sh (no WiFi interface on 192.168.1.x)"
fi

# ── 4. WireGuard VPN setup ────────────────────────────────────────────────────

echo ""
echo "=== WireGuard VPN setup ==="

# Install WireGuard if not present
if ! command -v wg &>/dev/null; then
    echo "Installing WireGuard..."
    sudo apt install -y wireguard wireguard-tools
fi

# Robot WireGuard public keys
source "$REPO/robots_wg.conf"

# Get or generate dev PC private key
if ip link show wg0 &>/dev/null 2>&1; then
    DEV_PRIV=$(echo '123456' | sudo -kS wg showconf wg0 2>/dev/null | awk '/PrivateKey/ {print $3}')
    echo "wg0 already running — reusing existing keypair."
elif [ -f /etc/wireguard/wg0.conf ]; then
    DEV_PRIV=$(echo '123456' | sudo -kS awk '/PrivateKey/ {print $3}' /etc/wireguard/wg0.conf 2>/dev/null)
    echo "Reusing existing WireGuard keypair."
else
    DEV_PRIV=$(wg genkey)
    echo "Generated new WireGuard keypair."
fi
DEV_PUB=$(echo "$DEV_PRIV" | wg pubkey)
echo "Dev PC public key : $DEV_PUB"

# Write /etc/wireguard/wg0.conf
cat > /tmp/topstar_wg0.conf << EOF
[Interface]
Address = ${DEV_WG_IP}/24
PrivateKey = ${DEV_PRIV}
ListenPort = 51820

[Peer]
# Robot 1 Computer A
PublicKey = ${ROBOT1_WG_PUB}
Endpoint = ${ROBOT1_WG_ENDPOINT}
AllowedIPs = ${ROBOT1_WG_IP}/32
PersistentKeepalive = 25

[Peer]
# Robot 2 Computer A
PublicKey = ${ROBOT2_WG_PUB}
Endpoint = ${ROBOT2_WG_ENDPOINT}
AllowedIPs = ${ROBOT2_WG_IP}/32
PersistentKeepalive = 25
EOF
sudo cp /tmp/topstar_wg0.conf /etc/wireguard/wg0.conf
sudo chmod 600 /etc/wireguard/wg0.conf
rm /tmp/topstar_wg0.conf

# Enable and start (or reload peers without dropping the interface)
sudo systemctl enable wg-quick@wg0
if ip link show wg0 &>/dev/null 2>&1; then
    sudo wg syncconf wg0 <(sudo wg-quick strip wg0)
    echo "wg0 config reloaded."
else
    sudo systemctl start wg-quick@wg0
    echo "wg0 started."
fi

# Register this dev PC's key on the currently reachable robot
echo -n "Registering dev PC with robot at 192.168.37.10 ... "
if ping -c 1 -W 2 192.168.37.10 &>/dev/null; then
    OLD_PUB=$(sshpass -p '123456' ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
        test@192.168.37.10 \
        "echo '123456' | sudo -kS wg show wg0 allowed-ips 2>/dev/null \
         | grep '10\.0\.0\.1/32' | awk '{print \$1}'" 2>/dev/null || true)
    if [ -n "$OLD_PUB" ] && [ "$OLD_PUB" != "$DEV_PUB" ]; then
        sshpass -p '123456' ssh test@192.168.37.10 \
            "echo '123456' | sudo -kS wg set wg0 peer ${OLD_PUB} remove" 2>/dev/null || true
    fi
    sshpass -p '123456' ssh test@192.168.37.10 \
        "echo '123456' | sudo -kS wg set wg0 peer ${DEV_PUB} \
         allowed-ips 10.0.0.1/32 persistent-keepalive 25 && \
         echo '123456' | sudo -kS wg-quick save wg0" && \
        echo "OK" || echo "FAILED"
else
    echo "robot not reachable."
    echo "  Switch to each robot's network and run: bash $REPO/register_wireguard.sh"
fi

# Generate WireGuard setup scripts
write_wg_setup_script() {
    local file="$1"
    local label="$2"
    local domain="$3"
    local robot_wg_ip="$4"
    cat > "$file" << SCRIPT
#!/bin/bash
echo "Setup topstar ros2 environment ($label)"
source /opt/ros/humble/setup.bash
source \$HOME/topstar_ros2/cyclonedds_ws/install/setup.bash
if [ -f "\$HOME/topstar_ros2/example/install/setup.bash" ]; then
    source \$HOME/topstar_ros2/example/install/setup.bash
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=$domain
# WireGuard path: wg0 virtual subnet — reliable DDS discovery without cross-subnet routing.
# IMPORTANT: keep CYCLONEDDS_URI on one line — CycloneDDS fails to parse multiline env vars.
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="wg0" priority="default" multicast="default"/></Interfaces><MaxMessageSize>1386B</MaxMessageSize></General><Discovery><Peers><Peer Address="$robot_wg_ip"/></Peers></Discovery></Domain></CycloneDDS>'
# Stop any stale ros2 daemon (graceful stop cleans socket files; pkill leaves them and causes !rclpy.ok() errors)
ros2 daemon stop 2>/dev/null || true
SCRIPT
    chmod +x "$file"
    echo "  Written: $file"
}

write_wg_setup_script "$REPO/setup_wg_r1.sh" "Robot 1, domain 1, WireGuard" 1 "$ROBOT1_WG_IP"
write_wg_setup_script "$REPO/setup_wg_r2.sh" "Robot 2, domain 2, WireGuard" 2 "$ROBOT2_WG_IP"

# ── 5. Done ───────────────────────────────────────────────────────────────────

echo ""
echo "=== Done ==="
echo ""
echo "Recommended scripts (WireGuard — most reliable):"
echo "  source ~/topstar_ros2/setup_wg_r1.sh    # Robot 1"
echo "  source ~/topstar_ros2/setup_wg_r2.sh    # Robot 2"
echo ""
echo "Fallback scripts (WiFi):"
echo "  source ~/topstar_ros2/setup_r1.sh       # Robot 1, WiFi"
echo "  source ~/topstar_ros2/setup.sh          # Robot 2, WiFi"
echo ""
echo "When connecting to a robot for the first time with this dev PC:"
echo "  bash ~/topstar_ros2/register_wireguard.sh"
echo ""
echo "To persist a default, add one of the source lines above to ~/.bashrc."
