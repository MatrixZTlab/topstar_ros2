#!/bin/bash
# Run this script on a new dev PC after cloning the topstar_ros2 repo.
# It detects your network interfaces, sets up routing to subnet-37 persistently,
# and generates the four robot setup scripts for this machine's interface names.
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

# ── 3. Generate setup scripts ─────────────────────────────────────────────────

echo ""
echo "=== Generating setup scripts in $REPO ==="

# Boilerplate shared by all four scripts (as a function body template)
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
# Kill stale daemons that may have started with wrong rmw/domain
pkill -f 'ros2-daemon.*rmw-implementation rmw_fastrtps' 2>/dev/null || true
SCRIPT
    chmod +x "$file"
    echo "  Written: $file"
}

# Wired URIs — cross-subnet discovery requires explicit unicast peers
WIRED_PEERS='<Peer Address="192.168.37.10"/><Peer Address="192.168.37.11"/><Peer Address="192.168.36.10"/><Peer Address="192.168.36.40"/>'
WIRED_URI_BASE="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$WIRED_IF\" priority=\"default\" multicast=\"default\"/></Interfaces></General><Internal><MaxMessageSize>1438B</MaxMessageSize></Internal><Discovery><Peers>$WIRED_PEERS</Peers></Discovery></Domain></CycloneDDS>"

write_setup_script "$REPO/setup_wired.sh" "Robot 2, domain 2, wired" 2 \
    "$WIRED_URI_BASE" \
    "Wired path (enp131s0 → subnet-36 → B → subnet-37 → A). Multicast doesn't cross subnets; unicast peers required."

write_setup_script "$REPO/setup_wired_r1.sh" "Robot 1, domain 1, wired" 1 \
    "$WIRED_URI_BASE" \
    "Wired path (enp131s0 → subnet-36 → B → subnet-37 → A). Multicast doesn't cross subnets; unicast peers required."

# WiFi URIs — same subnet as A's wlp4s0, so multicast works; only need A's WiFi IP as peer
if [ -n "$WIFI_IF" ]; then
    WIFI_URI_R2="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$WIFI_IF\" priority=\"default\" multicast=\"default\"/></Interfaces></General><Internal><MaxMessageSize>1438B</MaxMessageSize></Internal><Discovery><Peers><Peer Address=\"192.168.1.12\"/></Peers></Discovery></Domain></CycloneDDS>"
    WIFI_URI_R1="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$WIFI_IF\" priority=\"default\" multicast=\"default\"/></Interfaces></General><Internal><MaxMessageSize>1438B</MaxMessageSize></Internal><Discovery><Peers><Peer Address=\"192.168.1.11\"/></Peers></Discovery></Domain></CycloneDDS>"

    write_setup_script "$REPO/setup.sh" "Robot 2, domain 2, WiFi" 2 \
        "$WIFI_URI_R2" \
        "WiFi path: A's wlp4s0 is 192.168.1.12 — same subnet, multicast + unicast peer."

    write_setup_script "$REPO/setup_r1.sh" "Robot 1, domain 1, WiFi" 1 \
        "$WIFI_URI_R1" \
        "WiFi path: A's wlp4s0 is 192.168.1.11 — same subnet, multicast + unicast peer."
else
    echo "  Skipped: setup.sh and setup_r1.sh (no WiFi interface on 192.168.1.x)"
fi

# ── 4. Done ───────────────────────────────────────────────────────────────────

echo ""
echo "=== Done ==="
echo ""
echo "Source the appropriate script to start working:"
echo "  source ~/topstar_ros2/setup.sh          # Robot 2, WiFi"
echo "  source ~/topstar_ros2/setup_wired.sh    # Robot 2, wired"
echo "  source ~/topstar_ros2/setup_r1.sh       # Robot 1, WiFi"
echo "  source ~/topstar_ros2/setup_wired_r1.sh # Robot 1, wired"
echo ""
echo "To persist a default, add one of the above to ~/.bashrc."
