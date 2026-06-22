# Robot Network Setup

## Overview

The robot runs two onboard computers connected by a dedicated wired subnet. A development
workstation connects to the robot via a second subnet on Computer B.

```
Dev PC(s)                   Computer B                 Computer A
192.168.36.x  ── subnet36 ──  192.168.36.10            192.168.37.10
                               192.168.37.11  ── subnet37 ──
```

| Machine | Role | Subnet 36 IP | Subnet 37 IP |
|---|---|---|---|
| Computer A | Motion control, ROS2 nodes | — | 192.168.37.10 |
| Computer B | User dev (Jetson, camera, etc.) | 192.168.36.10 | 192.168.37.11 |
| Dev PC | Development / monitoring | 192.168.36.x | — |

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

### ROS2 / DDS Discovery

ROS2's default DDS (CycloneDDS) uses UDP multicast for node discovery, which does not
cross subnets. All machines are configured with a `cyclone_peers.xml` that lists
explicit unicast peer addresses, bypassing multicast. A common `ROS_DOMAIN_ID=2` ties
them into one logical ROS2 network. The second robot on the same WiFi uses
`ROS_DOMAIN_ID=1` to avoid cross-robot topic pollution.

---

## Setting Up a New Dev PC

Follow these steps on any new computer that connects to subnet 36 via Computer B.

### 1. Verify connectivity to B

```bash
ping 192.168.36.10
```

You should have a static IP on subnet 36 (e.g. `192.168.36.41`). If DHCP is in use,
note your assigned IP — you will need it in step 4.

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

Create `~/cyclone_peers.xml` with your machine's subnet-36 IP added to the peer list:

```xml
<CycloneDDS>
  <Domain>
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

### 6. Update peer lists on existing machines

CycloneDDS unicast discovery is bidirectional — existing machines must also know about
the new peer. Add the new IP to `~/cyclone_peers.xml` on every other dev PC, and to
`/etc/cyclonedds/config.xml` on Computer A (requires sudo):

```bash
# On Computer A — edit /etc/cyclonedds/config.xml and add:
#   <Peer Address="192.168.36.XX"/>
# then reload any running ROS2 nodes.

# On Computer B — edit ~/cyclone_peers.xml and add the same line.
```

### 7. Verify ROS2 discovery

With nodes running on Computer A:

```bash
ros2 node list     # should show nodes from A
ros2 topic list    # should show topics from A
```

A quick smoke test using the demo talker:

```bash
# On Computer A:
ros2 run demo_nodes_cpp talker

# On new dev PC:
ros2 topic echo /chatter --once
```

---

## Reference: Computer B configuration

B has IP forwarding enabled persistently:

```
/etc/sysctl.d/99-ip-forward.conf
  net.ipv4.ip_forward=1
```

B's CycloneDDS peer config: `~/cyclone_peers.xml`
B's ROS2 environment: `~/.bashrc` sources `/opt/ros/humble/setup.bash` with
`CYCLONEDDS_URI` and `ROS_DOMAIN_ID=2`.

## Reference: Computer A configuration

A's CycloneDDS config: `/etc/cyclonedds/config.xml` (referenced by `CYCLONEDDS_URI`
in `~/.bashrc`). It binds CycloneDDS to both `eno1` (subnet 37) and `wlp4s0` (WiFi).
The persistent route to subnet 36 is stored in the `netplan-eno1` NetworkManager
connection profile.
