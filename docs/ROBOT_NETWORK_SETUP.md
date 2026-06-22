# Robot Network Setup

## Overview

The robot runs two onboard computers connected by a dedicated wired subnet. A development
workstation connects to the robot via a second subnet on Computer B.

```
Dev PC(s)                   Computer B                 Computer A
192.168.36.x  ── subnet36 ──  192.168.36.10            192.168.37.10
                               192.168.37.11  ── subnet37 ──
                               (also on WiFi 192.168.110.x)
```

| Machine | Role | Subnet 36 IP | Subnet 37 IP | WiFi IP |
|---|---|---|---|---|
| Computer A | Motion control, ROS2 bridge | — | 192.168.37.10 (eno1) | 192.168.1.12 (wlp4s0) |
| Computer B | User dev (Jetson, camera, etc.) | 192.168.36.10 (lan2) | 192.168.37.11 (lan1) | — |
| Dev PC | Development / monitoring | 192.168.36.x | — | 192.168.1.x |

Computer B acts as a router between the two subnets (IP forwarding enabled), so dev
PCs on subnet 36 can reach Computer A on subnet 37 and participate in the same ROS2
network.

---

## How It Works

### IP Routing

B forwards packets between the two subnets. Each machine has a static route pointing
to B as the gateway for the other subnet:

- **Dev PC → A**: route `192.168.37.0/24 via 192.168.36.10`
- **A → Dev PC**: route `192.168.36.0/24 via 192.168.37.11`

B itself requires no added routes — it has direct connections to both subnets.

### ROS2 / DDS Discovery

ROS2's default DDS (CycloneDDS) uses UDP multicast for node discovery, which does not
cross subnets. All machines are configured with a CycloneDDS XML config that:

- Lists explicit unicast peers for cross-subnet discovery
- Restricts DDS to the relevant wired interfaces (avoids WiFi, cellular, USB bridge)
- Sets `MaxMessageSize=1438B` to stay within B's `lan1` MTU of 1466

A common `ROS_DOMAIN_ID=2` ties all machines into one logical ROS2 network. The second
robot on the same WiFi uses `ROS_DOMAIN_ID=1` to avoid cross-robot topic pollution.

### ROS2 Bridge on Computer A

The robot's DDS bridge (`topstar_bridge_v2`) runs as a systemd service. Its DDS
interface selection is controlled entirely by `/etc/cyclonedds/config.xml` (the
`--network_interface` flag must be absent — if present, it overrides the config file
and restricts DDS to a single interface). The config binds DDS to both `eno1` (wired,
reachable from B and dev PCs via routing) and `wlp4s0` (WiFi, reachable from dev PCs
on the same WiFi).

### MTU Constraint

B's `lan1` interface (subnet 37) has a non-standard MTU of **1466** bytes. Without
mitigation, DDS packets from A (MTU 1500) cause "sequence size exceeds remaining
buffer" parse errors on B. The `MaxMessageSize=1438B` setting (1466 − 20 IP − 8 UDP)
prevents this by keeping all RTPS messages within the path MTU.

---

## Persistent Configuration Details

### Computer B — IP Forwarding

```
/etc/sysctl.d/99-ip-forward.conf
  net.ipv4.ip_forward=1
```

### Computer A — Static Route

The route to subnet 36 is defined in netplan — **not** via `nmcli`, because netplan
regenerates NetworkManager connection files on reboot and wipes `nmcli` changes:

```yaml
# /etc/netplan/01-network-manager-all.yaml  (relevant section)
    eno1:
      addresses:
        - 192.168.37.10/24
      routes:
        - to: 192.168.36.0/24
          via: 192.168.37.11
```

### Computer A — CycloneDDS Config

`/etc/cyclonedds/config.xml` (referenced by `CYCLONEDDS_URI` in `~/.bashrc`):

```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="eno1"/>
        <NetworkInterface name="wlp4s0"/>
      </Interfaces>
    </General>
    <Internal>
      <MaxMessageSize>1438B</MaxMessageSize>
    </Internal>
    <Discovery>
      <Peers>
        <Peer Address="192.168.37.10"/>
        <Peer Address="192.168.37.11"/>
        <Peer Address="192.168.36.10"/>
        <Peer Address="192.168.36.40"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

### Computer A — Bridge Service

`/etc/systemd/system/topstar_bridge_v2.service` must **not** contain
`--network_interface`. If a robot software update restores that flag, remove it and
restart the service:

```bash
sudo sed -i 's/ --network_interface=[^ ]*//' /etc/systemd/system/topstar_bridge_v2.service
sudo systemctl daemon-reload && sudo systemctl restart topstar_bridge_v2.service
```

### Computer B — CycloneDDS Config

`~/cyclone_peers.xml`:

```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="lan1"/>
        <NetworkInterface name="lan2"/>
      </Interfaces>
    </General>
    <Internal>
      <MaxMessageSize>1438B</MaxMessageSize>
    </Internal>
    <Discovery>
      <Peers>
        <Peer Address="192.168.37.10"/>
        <Peer Address="192.168.37.11"/>
        <Peer Address="192.168.36.10"/>
        <Peer Address="192.168.36.40"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

---

## Setting Up a New Dev PC

Follow these steps on any new computer that connects to subnet 36 via Computer B.

### 1. Verify connectivity to B

```bash
ping 192.168.36.10
```

You should have a static IP on subnet 36 (e.g. `192.168.36.41`). If DHCP is in use,
note your assigned IP — you will need it in steps 4 and 6.

### 2. Add a static route to subnet 37

```bash
# Apply immediately
sudo ip route add 192.168.37.0/24 via 192.168.36.10

# Verify
ping 192.168.37.10   # should reach Computer A
```

### 3. Make the route persistent (NetworkManager)

Find the connection name for your subnet-36 interface:

```bash
nmcli -t -f NAME,DEVICE,STATE connection show --active
```

Then persist the route (replace `<connection-name>` with the name from above):

```bash
sudo nmcli connection modify "<connection-name>" +ipv4.routes "192.168.37.0/24 192.168.36.10"
sudo nmcli connection up "<connection-name>"
```

### 4. Create the CycloneDDS peer config

Create `~/cyclone_peers.xml` with your machine's subnet-36 IP added to the peer list.
Include `MaxMessageSize` to handle B's MTU 1466 constraint on lan1:

```xml
<CycloneDDS>
  <Domain>
    <Internal>
      <MaxMessageSize>1438B</MaxMessageSize>
    </Internal>
    <Discovery>
      <Peers>
        <Peer Address="192.168.37.10"/>   <!-- Computer A -->
        <Peer Address="192.168.37.11"/>   <!-- Computer B (subnet-37) -->
        <Peer Address="192.168.36.10"/>   <!-- Computer B (subnet-36) -->
        <Peer Address="192.168.36.40"/>   <!-- existing dev PC -->
        <Peer Address="192.168.36.XX"/>   <!-- this new machine -->
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

Replace `192.168.36.XX` with this machine's actual IP.

### 5. Add environment variables to `~/.bashrc`

```bash
cat >> ~/.bashrc << 'EOF'
source /opt/ros/humble/setup.bash
export CYCLONEDDS_URI=file:///home/$USER/cyclone_peers.xml
export ROS_DOMAIN_ID=2
EOF

source ~/.bashrc
```

> **Note:** `~/.bashrc` is only sourced in interactive shells. Scripts and SSH
> non-interactive sessions must set `CYCLONEDDS_URI` and `ROS_DOMAIN_ID` explicitly,
> or source the file manually.

### 6. Update peer lists on existing machines

CycloneDDS unicast discovery is bidirectional — existing machines must also know about
the new peer. Add `<Peer Address="192.168.36.XX"/>` to:

- `~/cyclone_peers.xml` on every other dev PC
- `/etc/cyclonedds/config.xml` on Computer A (requires sudo), then restart the bridge:

```bash
sudo systemctl restart topstar_bridge_v2.service
```

- `~/cyclone_peers.xml` on Computer B

### 7. Verify ROS2 discovery

```bash
ros2 topic list    # should include /lowstate, /bms/state, /api/* etc.
```

A quick smoke test:

```bash
# On Computer A:
ros2 run demo_nodes_cpp talker

# On new dev PC:
ros2 topic echo /chatter --once
```

---

## Troubleshooting

**"sequence size exceeds remaining buffer" errors on B**
CycloneDDS MTU mismatch. Ensure `<MaxMessageSize>1438B</MaxMessageSize>` is present
in `~/cyclone_peers.xml` on B and in `/etc/cyclonedds/config.xml` on A, then restart
the bridge service on A.

**Robot topics not visible after reboot**
Check in order:
1. `ping 192.168.37.10` — if it fails, B's IP forwarding or the static routes are
   down. Check `cat /proc/sys/net/ipv4/ip_forward` on B and `ip route show` on A.
2. Ensure `CYCLONEDDS_URI` and `ROS_DOMAIN_ID=2` are set (`printenv | grep ROS`).
3. `sudo systemctl status topstar_bridge_v2.service` on A — confirm the bridge is
   active and has no `--network_interface` flag in its `ExecStart` line.

**Topics visible from dev PC (WiFi) but not from B (wired only)**
A's bridge is likely restricted to a single interface. Check:
```bash
# On A:
sudo systemctl cat topstar_bridge_v2.service | grep network_interface
```
If the flag is present, remove it as described in the Bridge Service section above.
