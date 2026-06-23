# Robot Network Setup

## Overview

The robot runs two onboard computers connected by a dedicated wired subnet. A development
workstation connects to the robot via a second subnet on Computer B, with a WireGuard VPN
tunnel providing reliable ROS2 DDS discovery.

```
Dev PC(s)                   Computer B                 Computer A
192.168.36.x  ── subnet36 ──  192.168.36.10            192.168.37.10
10.0.0.1      ══ WireGuard ══════════════════════════  10.0.0.2 / 10.0.0.3
                               192.168.37.11  ── subnet37 ──
```

| Machine | Role | Subnet 36 IP | Subnet 37 IP | WiFi IP | WireGuard IP |
|---|---|---|---|---|---|
| Computer A (Robot 1) | Motion control, ROS2 bridge | — | 192.168.37.10 (eno1) | 192.168.1.11 (wlp4s0) | 10.0.0.2 (wg0) |
| Computer A (Robot 2) | Motion control, ROS2 bridge | — | 192.168.37.10 (eno1) | 192.168.1.12 (wlp4s0) | 10.0.0.3 (wg0) |
| Computer B | User dev (Jetson, camera, etc.) | 192.168.36.10 (lan2) | 192.168.37.11 (lan1) | — | — |
| Dev PC | Development / monitoring | 192.168.36.x | — | 192.168.1.x | 10.0.0.1 (wg0) |

Both robots share the same wired subnet IPs and are used exclusively (one at a time).
Computer B acts as a router between the two wired subnets (IP forwarding enabled).

ROS2 domain IDs separate the two robots:

| Robot | `ROS_DOMAIN_ID` |
|---|---|
| Robot 1 | 1 |
| Robot 2 | 2 |

---

## How It Works

### WireGuard VPN (recommended)

DDS multicast does not cross subnet boundaries, and cross-subnet unicast discovery is
fragile (multi-locator conflicts, MTU mismatches, routing timing). The recommended
approach is a WireGuard VPN tunnel that places the dev PC and Computer A on the same
virtual subnet (10.0.0.0/24).

- Dev PC sends SPDP unicast to Computer A's WireGuard IP (10.0.0.2 or 10.0.0.3)
- Both peers are on the same /24 — no routing through B needed for DDS traffic
- The tunnel is encrypted UDP, carried over the physical wired path (port 51820)
- `wg-quick@wg0` is enabled as a systemd service on Computer A — survives reboots

Both robots share the same wired IP (192.168.37.10) but have different WireGuard IPs.
All dev PCs share the same WireGuard IP (10.0.0.1) since only one connects at a time.

### WiFi (fallback)

Computer A's `wlp4s0` and the dev PC's WiFi interface are on the same 192.168.1.x
subnet. Multicast discovery works directly. The WiFi setup scripts (`setup_r1.sh`,
`setup.sh`) use this path.

### IP Routing

B forwards packets between the two wired subnets. Each machine has a static route
pointing to B as the gateway for the other subnet:

- **Dev PC → A**: route `192.168.37.0/24 via 192.168.36.10`
- **A → Dev PC**: route `192.168.36.0/24 via 192.168.37.11`

This routing is required for WireGuard handshake packets (port 51820 UDP) to reach
Computer A. Plain IP routing works reliably; only DDS discovery (UDP multicast/unicast
with specific locator negotiation) is unreliable cross-subnet.

### ROS2 Bridge on Computer A

The robot's DDS bridge (`topstar_bridge_v2`) runs as a systemd service. Its DDS
interface selection is controlled entirely by `/etc/cyclonedds/config.xml`. The config
binds DDS to `eno1` (wired), `wlp4s0` (WiFi), and `wg0` (WireGuard) so that dev PCs
can connect via any of these paths.

> **Important:** The `--network_interface` flag must be absent from the bridge service's
> `ExecStart` line. If a robot software update restores it, remove it:
> ```bash
> sudo sed -i 's/ --network_interface=[^ ]*//' /etc/systemd/system/topstar_bridge_v2.service
> sudo systemctl daemon-reload && sudo systemctl restart topstar_bridge_v2.service
> ```

### MTU Constraint

B's `lan1` interface (subnet 37) has a non-standard MTU of **1466** bytes. The
`MaxMessageSize=1438B` setting (1466 − 20 IP − 8 UDP) keeps RTPS messages within the
path MTU. The WireGuard interface (`wg0`) uses `MaxMessageSize=1386B` (WireGuard MTU).

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

`/etc/cyclonedds/config.xml` (referenced by `CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml`
in the bridge service environment):

```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="eno1"/>
        <NetworkInterface name="wlp4s0"/>
        <NetworkInterface name="wg0"/>
      </Interfaces>
      <MaxMessageSize>1438B</MaxMessageSize>
    </General>
    <Discovery>
      <Peers>
        <Peer Address="192.168.37.10"/>
        <Peer Address="192.168.37.11"/>
        <Peer Address="192.168.36.10"/>
        <Peer Address="192.168.36.40"/>
        <Peer Address="10.0.0.1"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

Note: `MaxMessageSize` belongs in `<General>`, not `<Internal>` — newer CycloneDDS
versions deprecated the `<Internal>` location.

### Computer A — WireGuard Config

`/etc/wireguard/wg0.conf` (managed by `wg-quick@wg0`, enabled at boot):

```ini
[Interface]
Address = 10.0.0.2/24          # 10.0.0.3/24 for Robot 2
PrivateKey = <robot_private_key>
ListenPort = 51820

[Peer]
# Dev PC (all dev PCs share 10.0.0.1; only one connects at a time)
PublicKey = <devpc_public_key>
AllowedIPs = 10.0.0.1/32
PersistentKeepalive = 25
```

The dev PC's public key is stored in `robots_wg.conf` in the repo. When a new dev PC
is set up, run `register_wireguard.sh` to update the peer (see below).

### Dev PC — WireGuard Config

`/etc/wireguard/wg0.conf` (generated by `devpc_setup.sh`):

```ini
[Interface]
Address = 10.0.0.1/24
PrivateKey = <devpc_private_key>
ListenPort = 51820

[Peer]
# Robot 1 Computer A
PublicKey = xIHu3bWA+kLKelWWmYnIn3ArY8Eg5N2k9yGRsIp0R1I=
Endpoint = 192.168.37.10:51820
AllowedIPs = 10.0.0.2/32
PersistentKeepalive = 25

[Peer]
# Robot 2 Computer A
PublicKey = 01d+Yq8RYZ/ReOWMkWBQDIsUs2z29qt8gg2hmF1liHY=
Endpoint = 192.168.37.10:51820
AllowedIPs = 10.0.0.3/32
PersistentKeepalive = 25
```

Robot public keys are also stored in `robots_wg.conf` for reference.

### Computer B — setup.sh

`~/topstar_ros2/setup.sh` on B must set the correct domain and stop any stale daemon:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=2
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lan1" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
ros2 daemon stop 2>/dev/null || true
```

---

## Setting Up a New Dev PC

Connect the dev PC to subnet 36 (wired, via Computer B) and assign a static IP
(e.g. `192.168.36.41`). Then run the automated setup script:

```bash
git clone <repo_url> ~/topstar_ros2
cd ~/topstar_ros2
bash devpc_setup.sh
```

`devpc_setup.sh` does the following automatically:
1. Detects the subnet-36 wired interface and optional WiFi interface
2. Adds the static route `192.168.37.0/24 via 192.168.36.10` and persists it via nmcli
3. Generates WiFi setup scripts (`setup.sh`, `setup_r1.sh`) with the detected interface names
4. Installs WireGuard, generates a keypair, writes `/etc/wireguard/wg0.conf` with both
   robots as peers (using public keys from `robots_wg.conf`), and starts `wg-quick@wg0`
5. Registers this dev PC's public key on the currently connected robot

### Registering with the second robot

Since both robots share the same wired IP, `devpc_setup.sh` can only register with one
robot per run. Switch to the other robot's network and run:

```bash
bash ~/topstar_ros2/register_wireguard.sh
```

This replaces the old dev PC peer on that robot with the new public key.

### Using the setup scripts

After `devpc_setup.sh` completes, source the appropriate script in each shell session:

```bash
source ~/topstar_ros2/setup_wg_r1.sh    # Robot 1 via WireGuard (recommended)
source ~/topstar_ros2/setup_wg_r2.sh    # Robot 2 via WireGuard (recommended)
source ~/topstar_ros2/setup_r1.sh       # Robot 1 via WiFi (fallback)
source ~/topstar_ros2/setup.sh          # Robot 2 via WiFi (fallback)
```

To persist a default, add one of the above `source` lines to `~/.bashrc`.

---

## Troubleshooting

**`!rclpy.ok()` error from `ros2 topic list`**
A stale `ros2 daemon` is running with the wrong environment (wrong domain, wrong rmw,
or started before the setup script was sourced). The setup scripts call `ros2 daemon stop`
to handle this, but if the error persists, run manually:
```bash
ros2 daemon stop && sleep 1 && ros2 topic list
```

**WireGuard tunnel not coming up after reboot**
Verify `wg-quick@wg0` is enabled on both ends:
```bash
# Dev PC
sudo systemctl enable wg-quick@wg0

# Computer A (via SSH)
sshpass -p '123456' ssh test@192.168.37.10 'systemctl is-enabled wg-quick@wg0'
```

**WireGuard connected (ping 10.0.0.2 works) but `ros2 topic list` only shows local topics**
Computer A's `/etc/cyclonedds/config.xml` may be missing `wg0` in its interface list,
or the dev PC's public key may not be registered. Re-run `register_wireguard.sh`.

**Robot topics not visible after reboot**
Check in order:
1. `ping 10.0.0.2` (or `10.0.0.3`) — if it fails, WireGuard is down; check both ends with `sudo systemctl status wg-quick@wg0`
2. `ping 192.168.37.10` — if it fails, B's IP forwarding or static routes are down
3. `sudo systemctl status topstar_bridge_v2.service` on A — confirm the bridge is active

**Bridge service fails to start or exits immediately**
Check the bridge log:
```bash
sshpass -p '123456' ssh test@192.168.37.10 'journalctl -u topstar_bridge_v2.service -n 30 --no-pager'
```

**Python/rclpy subscriber on B receives no data even though C++ nodes work**
A stale `ros2-daemon` running with Fast-DDS (rmw_fastrtps_cpp) is likely competing for
the SPDP multicast port. Detect with:
```bash
pgrep -a ros2-daemon    # look for entries with "rmw_fastrtps"
```
Fix by stopping the daemon (`ros2 daemon stop`) and re-sourcing `setup.sh`.

**Topics visible from dev PC (WiFi) but not via WireGuard**
Confirm Computer A's CycloneDDS config includes the `wg0` interface and `10.0.0.1` peer.
If the config was changed, restart the bridge:
```bash
sshpass -p '123456' ssh test@192.168.37.10 \
  "echo '123456' | sudo -S systemctl restart topstar_bridge_v2.service"
```
