/**
 * @file mujoco_ros2_bridge.cpp
 * @brief DDS relay for mirroring real-robot H2 commands and state into MuJoCo.
 *
 * Reads raw DDS `rt/lowcmd` (and optionally `rt/lowstate`) from the real-robot
 * side on one interface (typically `eno1`) and republishes to MuJoCo on another
 * interface (typically `lo`):
 *
 *   rt/lowcmd   (eno1 → lo)  : command mirror, drives MuJoCo actuators
 *   rt/lowstate (eno1 → lo)  : state mirror, published as rt/lowstate_robot
 *                               for topstar_mujoco --lowstate kinematic display
 *
 * CycloneDDS interface binding is process-wide via CYCLONEDDS_URI, so this
 * executable forks into two processes:
 *   child  -> ingress reader bound to the robot-facing interface
 *   parent -> egress writer bound to the MuJoCo-facing interface
 *
 * LowCmd and LowState samples are forwarded over separate local pipes.
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include <cerrno>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <dds/dds.h>
#include "topstar_hg.h"

static const char *LOWCMD_TOPIC        = "rt/lowcmd";
static const char *LOWSTATE_TOPIC      = "rt/lowstate";
static const char *LOWSTATE_ROBOT_TOPIC = "rt/lowstate_robot";

static constexpr int PIPE_READ_END  = 0;
static constexpr int PIPE_WRITE_END = 1;
static constexpr int COMMAND_STALE_TIMEOUT_MS = 200;

static std::atomic<bool> g_running{true};

static void signal_handler(int) { g_running = false; }

static bool extract_iface_from_cyclonedds_uri(const char *uri_cstr, std::string &iface_out)
{
    if (uri_cstr == nullptr || uri_cstr[0] == '\0') return false;

    const std::string uri(uri_cstr);
    size_t ni_pos = uri.find("NetworkInterface");
    if (ni_pos == std::string::npos) return false;

    size_t name_pos = uri.find("name=\"", ni_pos);
    char quote = '\0';
    if (name_pos != std::string::npos) {
        quote = '"';
    } else {
        name_pos = uri.find("name='", ni_pos);
        if (name_pos != std::string::npos) {
            quote = '\'';
        }
    }
    if (name_pos == std::string::npos) return false;

    size_t value_start = name_pos + 6;
    size_t value_end = uri.find(quote, value_start);
    if (value_end == std::string::npos || value_end <= value_start) return false;

    iface_out = uri.substr(value_start, value_end - value_start);
    return !iface_out.empty();
}

static uint32_t calculate_crc32(uint32_t *ptr, uint32_t len)
{
    uint32_t xbit = 0, data = 0, crc32 = 0xFFFFFFFF;
    const uint32_t poly = 0x04c11db7;
    for (uint32_t i = 0; i < len; ++i) {
        xbit = 1u << 31;
        data = ptr[i];
        for (uint32_t b = 0; b < 32; ++b) {
            if (crc32 & 0x80000000u) { crc32 <<= 1; crc32 ^= poly; }
            else                     { crc32 <<= 1; }
            if (data & xbit) crc32 ^= poly;
            xbit >>= 1;
        }
    }
    return crc32;
}

static void configure_cyclonedds_interface(const std::string &interface_name,
                                           bool allow_existing_env,
                                           const char *role)
{
    const char *existing_uri = std::getenv("CYCLONEDDS_URI");
    if (allow_existing_env && existing_uri != nullptr && existing_uri[0] != '\0') {
        fprintf(stdout,
                "[DDS][%s] Using existing CYCLONEDDS_URI from environment; "
                "ignoring interface override '%s'\n",
                role,
                interface_name.c_str());
        return;
    }

    if (!allow_existing_env && existing_uri != nullptr && existing_uri[0] != '\0') {
        fprintf(stdout,
                "[DDS][%s] Overriding inherited CYCLONEDDS_URI with interface '%s'\n",
                role,
                interface_name.c_str());
    }

    std::string uri =
        "<CycloneDDS><Domain><General>"
        "<Interfaces><NetworkInterface name='" + interface_name + "'/></Interfaces>"
        "</General></Domain></CycloneDDS>";
    if (setenv("CYCLONEDDS_URI", uri.c_str(), 1) != 0) {
        fprintf(stderr, "[DDS][%s] Failed to set CYCLONEDDS_URI for interface '%s'\n", role, interface_name.c_str());
        return;
    }
    fprintf(stdout, "[DDS][%s] Bound CycloneDDS to interface '%s'\n", role, interface_name.c_str());
}

static bool write_full(int fd, const void *buffer, size_t size)
{
    const char *cursor = static_cast<const char *>(buffer);
    size_t remaining = size;
    while (remaining > 0 && g_running) {
        ssize_t written = write(fd, cursor, remaining);
        if (written < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        cursor    += written;
        remaining -= static_cast<size_t>(written);
    }
    return remaining == 0;
}

static bool drain_latest_lowcmd(int fd, topstar_hg_msg_dds__LowCmd_ &latest, bool &received_any)
{
    bool updated = false;
    while (g_running) {
        topstar_hg_msg_dds__LowCmd_ candidate{};
        ssize_t n = read(fd, &candidate, sizeof(candidate));
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            return false;
        }
        if (n == 0) break;
        if (static_cast<size_t>(n) != sizeof(candidate)) continue;
        latest = candidate;
        received_any = true;
        updated = true;
    }
    return updated;
}

static bool drain_latest_lowstate(int fd, topstar_hg_msg_dds__LowState_ &latest, bool &received_any)
{
    bool updated = false;
    while (g_running) {
        topstar_hg_msg_dds__LowState_ candidate{};
        ssize_t n = read(fd, &candidate, sizeof(candidate));
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            return false;
        }
        if (n == 0) break;
        if (static_cast<size_t>(n) != sizeof(candidate)) continue;
        latest = candidate;
        received_any = true;
        updated = true;
    }
    return updated;
}

// ── Ingress DDS readers (robot interface) ─────────────────────────────────────

class DdsLowCmdReader {
public:
    DdsLowCmdReader(const std::string &iface, bool allow_existing_env)
        : iface_(iface), allow_existing_env_(allow_existing_env) {}
    ~DdsLowCmdReader() { cleanup(); }

    bool initialize() {
        configure_cyclonedds_interface(iface_, allow_existing_env_, "Ingress");
        participant_ = dds_create_participant(0, NULL, NULL);
        if (participant_ < 0) { fprintf(stderr, "[Ingress] lowcmd participant failed (%d)\n", participant_); return false; }
        topic_ = dds_create_topic(participant_, &topstar_hg_msg_dds__LowCmd__desc, LOWCMD_TOPIC, NULL, NULL);
        if (topic_ < 0) { fprintf(stderr, "[Ingress] lowcmd topic failed (%d)\n", topic_); return false; }
        dds_qos_t *qos = dds_create_qos();
        dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
        dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
        dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
        reader_ = dds_create_reader(participant_, topic_, qos, NULL);
        dds_delete_qos(qos);
        if (reader_ < 0) { fprintf(stderr, "[Ingress] lowcmd reader failed (%d)\n", reader_); return false; }
        waitset_  = dds_create_waitset(participant_);
        if (waitset_ > 0) {
            condition_ = dds_create_readcondition(reader_, DDS_ANY_STATE);
            if (condition_ > 0) dds_waitset_attach(waitset_, condition_, 0);
        }
        fprintf(stdout, "[Ingress] listening for %s on %s\n", LOWCMD_TOPIC, iface_.c_str());
        return true;
    }

    bool take(topstar_hg_msg_dds__LowCmd_ &cmd) {
        void *samples[1] = {NULL};
        dds_sample_info_t infos[1];
        int32_t n = dds_take(reader_, samples, infos, 1, 1);
        bool ok = false;
        if (n > 0 && infos[0].valid_data) {
            auto *s = static_cast<topstar_hg_msg_dds__LowCmd_ *>(samples[0]);
            if (s) { cmd = *s; ok = true; }
        }
        if (n > 0) dds_return_loan(reader_, samples, n);
        return ok;
    }

    void wait_for_data() {
        if (waitset_ > 0) dds_waitset_wait(waitset_, NULL, 0, DDS_MSECS(1));
        else std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

private:
    void cleanup() {
        if (condition_ > 0) dds_delete(condition_);
        if (waitset_   > 0) dds_delete(waitset_);
        if (reader_    > 0) dds_delete(reader_);
        if (topic_     > 0) dds_delete(topic_);
        if (participant_ > 0) dds_delete(participant_);
    }
    std::string  iface_;
    bool allow_existing_env_;
    dds_entity_t participant_ = 0, topic_ = 0, reader_ = 0, waitset_ = 0, condition_ = 0;
};

class DdsLowStateReader {
public:
    explicit DdsLowStateReader(const std::string &iface) : iface_(iface) {}
    ~DdsLowStateReader() { cleanup(); }

    bool initialize() {
        // CYCLONEDDS_URI already set by DdsLowCmdReader in this process.
        participant_ = dds_create_participant(0, NULL, NULL);
        if (participant_ < 0) { fprintf(stderr, "[Ingress] lowstate participant failed (%d)\n", participant_); return false; }
        topic_ = dds_create_topic(participant_, &topstar_hg_msg_dds__LowState__desc, LOWSTATE_TOPIC, NULL, NULL);
        if (topic_ < 0) { fprintf(stderr, "[Ingress] lowstate topic failed (%d)\n", topic_); return false; }
        dds_qos_t *qos = dds_create_qos();
        dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
        dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
        dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
        reader_ = dds_create_reader(participant_, topic_, qos, NULL);
        dds_delete_qos(qos);
        if (reader_ < 0) { fprintf(stderr, "[Ingress] lowstate reader failed (%d)\n", reader_); return false; }
        fprintf(stdout, "[Ingress] listening for %s on %s\n", LOWSTATE_TOPIC, iface_.c_str());
        return true;
    }

    bool take(topstar_hg_msg_dds__LowState_ &state) {
        void *samples[1] = {NULL};
        dds_sample_info_t infos[1];
        int32_t n = dds_take(reader_, samples, infos, 1, 1);
        bool ok = false;
        if (n > 0 && infos[0].valid_data) {
            auto *s = static_cast<topstar_hg_msg_dds__LowState_ *>(samples[0]);
            if (s) { state = *s; ok = true; }
        }
        if (n > 0) dds_return_loan(reader_, samples, n);
        return ok;
    }

private:
    void cleanup() {
        if (reader_    > 0) dds_delete(reader_);
        if (topic_     > 0) dds_delete(topic_);
        if (participant_ > 0) dds_delete(participant_);
    }
    std::string  iface_;
    dds_entity_t participant_ = 0, topic_ = 0, reader_ = 0;
};

// ── Egress DDS writers (sim interface) ───────────────────────────────────────

class DdsLowCmdWriter {
public:
    explicit DdsLowCmdWriter(const std::string &iface) : iface_(iface) {}
    ~DdsLowCmdWriter() { cleanup(); }

    bool initialize() {
        configure_cyclonedds_interface(iface_, false, "Egress");
        participant_ = dds_create_participant(0, NULL, NULL);
        if (participant_ < 0) { fprintf(stderr, "[Egress] lowcmd participant failed (%d)\n", participant_); return false; }
        topic_ = dds_create_topic(participant_, &topstar_hg_msg_dds__LowCmd__desc, LOWCMD_TOPIC, NULL, NULL);
        if (topic_ < 0) { fprintf(stderr, "[Egress] lowcmd topic failed (%d)\n", topic_); return false; }
        dds_qos_t *qos = dds_create_qos();
        dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
        dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
        dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
        writer_ = dds_create_writer(participant_, topic_, qos, NULL);
        dds_delete_qos(qos);
        if (writer_ < 0) { fprintf(stderr, "[Egress] lowcmd writer failed (%d)\n", writer_); return false; }
        fprintf(stdout, "[Egress] publishing %s on %s\n", LOWCMD_TOPIC, iface_.c_str());
        return true;
    }

    bool write(topstar_hg_msg_dds__LowCmd_ &cmd) {
        cmd.crc = calculate_crc32(reinterpret_cast<uint32_t *>(&cmd),
                                  sizeof(cmd) / sizeof(uint32_t) - 1);
        return dds_write(writer_, &cmd) >= 0;
    }

private:
    void cleanup() {
        if (writer_    > 0) dds_delete(writer_);
        if (topic_     > 0) dds_delete(topic_);
        if (participant_ > 0) dds_delete(participant_);
    }
    std::string  iface_;
    dds_entity_t participant_ = 0, topic_ = 0, writer_ = 0;
};

class DdsLowStateWriter {
public:
    explicit DdsLowStateWriter(const std::string &iface) : iface_(iface) {}
    ~DdsLowStateWriter() { cleanup(); }

    bool initialize() {
        // CYCLONEDDS_URI already set by DdsLowCmdWriter in this process.
        participant_ = dds_create_participant(0, NULL, NULL);
        if (participant_ < 0) { fprintf(stderr, "[Egress] lowstate participant failed (%d)\n", participant_); return false; }
        topic_ = dds_create_topic(participant_, &topstar_hg_msg_dds__LowState__desc, LOWSTATE_ROBOT_TOPIC, NULL, NULL);
        if (topic_ < 0) { fprintf(stderr, "[Egress] lowstate_robot topic failed (%d)\n", topic_); return false; }
        dds_qos_t *qos = dds_create_qos();
        dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
        dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
        dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
        writer_ = dds_create_writer(participant_, topic_, qos, NULL);
        dds_delete_qos(qos);
        if (writer_ < 0) { fprintf(stderr, "[Egress] lowstate_robot writer failed (%d)\n", writer_); return false; }
        fprintf(stdout, "[Egress] publishing %s on %s\n", LOWSTATE_ROBOT_TOPIC, iface_.c_str());
        return true;
    }

    bool write(const topstar_hg_msg_dds__LowState_ &state) {
        return dds_write(writer_, &state) >= 0;
    }

private:
    void cleanup() {
        if (writer_    > 0) dds_delete(writer_);
        if (topic_     > 0) dds_delete(topic_);
        if (participant_ > 0) dds_delete(participant_);
    }
    std::string  iface_;
    dds_entity_t participant_ = 0, topic_ = 0, writer_ = 0;
};

// ── Process entry points ──────────────────────────────────────────────────────

static int run_ingress(const std::string &robot_interface,
                       bool allow_existing_env_uri,
                       int cmd_pipe_fd, int state_pipe_fd)
{
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    DdsLowCmdReader   cmd_reader(robot_interface, allow_existing_env_uri);
    DdsLowStateReader state_reader(robot_interface);
    if (!cmd_reader.initialize())   return 1;
    if (!state_reader.initialize()) return 1;

    bool warned_first_cmd = false;

    while (g_running) {
        topstar_hg_msg_dds__LowCmd_ cmd{};
        if (cmd_reader.take(cmd)) {
            if (!warned_first_cmd) {
                warned_first_cmd = true;
                fprintf(stderr, "[Ingress] received first %s sample\n", LOWCMD_TOPIC);
            }
            if (!write_full(cmd_pipe_fd, &cmd, sizeof(cmd))) {
                fprintf(stderr, "[Ingress] failed to forward lowcmd\n");
                break;
            }
        }

        topstar_hg_msg_dds__LowState_ state{};
        if (state_reader.take(state)) {
            if (!write_full(state_pipe_fd, &state, sizeof(state))) {
                fprintf(stderr, "[Ingress] failed to forward lowstate\n");
                break;
            }
        }

        cmd_reader.wait_for_data();
    }

    close(cmd_pipe_fd);
    close(state_pipe_fd);
    return 0;
}

static int run_egress(const std::string &sim_interface,
                      int cmd_pipe_fd, int state_pipe_fd)
{
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    auto set_nonblock = [](int fd) {
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    };
    set_nonblock(cmd_pipe_fd);
    set_nonblock(state_pipe_fd);

    DdsLowCmdWriter   cmd_writer(sim_interface);
    DdsLowStateWriter state_writer(sim_interface);
    if (!cmd_writer.initialize())   return 1;
    if (!state_writer.initialize()) return 1;

    topstar_hg_msg_dds__LowCmd_   latest_cmd{};
    topstar_hg_msg_dds__LowCmd_   zero_cmd{};
    topstar_hg_msg_dds__LowState_ latest_state{};
    bool has_cmd   = false;
    bool has_state = false;
    bool zero_sent = false;
    auto last_cmd_time = std::chrono::steady_clock::time_point{};

    while (g_running) {
        bool cmd_updated   = drain_latest_lowcmd  (cmd_pipe_fd,   latest_cmd,   has_cmd);
        bool state_updated = drain_latest_lowstate(state_pipe_fd, latest_state, has_state);

        if (cmd_updated) {
            last_cmd_time = std::chrono::steady_clock::now();
            zero_sent = false;
        }

        bool cmd_fresh = has_cmd &&
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - last_cmd_time).count() < COMMAND_STALE_TIMEOUT_MS;

        if (cmd_fresh) {
            cmd_writer.write(latest_cmd);
        } else if (!zero_sent) {
            memset(&zero_cmd, 0, sizeof(zero_cmd));
            cmd_writer.write(zero_cmd);
            zero_sent = true;
        }

        if (has_state && state_updated) {
            state_writer.write(latest_state);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    close(cmd_pipe_fd);
    close(state_pipe_fd);
    return 0;
}

static void print_usage(const char *prog)
{
    fprintf(stdout,
        "Usage: %s [OPTIONS]\n"
        "\n"
        "Mirror real-robot DDS traffic into MuJoCo.\n"
        "\n"
        "  rt/lowcmd   relayed from robot_interface → sim_interface\n"
        "  rt/lowstate relayed from robot_interface → sim_interface as rt/lowstate_robot\n"
        "              (used by topstar_mujoco --lowstate for kinematic mirror display)\n"
        "\n"
        "Options:\n"
        "  --robot_interface=IF  DDS interface for real-robot traffic (default: eno1)\n"
        "  --sim_interface=IF    DDS interface for MuJoCo traffic (default: lo)\n"
        "  --help                Show this help message\n",
        prog);
}

int main(int argc, char **argv)
{
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    std::string robot_interface = "eno1";
    std::string sim_interface   = "lo";
    bool robot_interface_explicit = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.rfind("--robot_interface=", 0) == 0) {
            robot_interface = arg.substr(18);
            robot_interface_explicit = true;
        } else if (arg == "--robot_interface" && i + 1 < argc) {
            robot_interface = argv[++i];
            robot_interface_explicit = true;
        } else if (arg.rfind("--sim_interface=", 0) == 0) {
            sim_interface = arg.substr(16);
        } else if (arg == "--sim_interface" && i + 1 < argc) {
            sim_interface = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        }
    }

    const char *existing_uri = std::getenv("CYCLONEDDS_URI");
    bool ingress_uses_existing_env = !robot_interface_explicit &&
                                     existing_uri != nullptr &&
                                     existing_uri[0] != '\0';
    if (ingress_uses_existing_env) {
        std::string iface_from_uri;
        if (extract_iface_from_cyclonedds_uri(existing_uri, iface_from_uri)) {
            robot_interface = iface_from_uri;
        }
    }

    fprintf(stdout, "=== H2 MuJoCo DDS Mirror Relay ===\n");
    fprintf(stdout, "Robot DDS interface : %s\n", robot_interface.c_str());
    fprintf(stdout, "MuJoCo DDS interface: %s\n", sim_interface.c_str());
    fprintf(stdout, "Relaying: %s → %s, %s → %s\n",
            LOWCMD_TOPIC, LOWCMD_TOPIC, LOWSTATE_TOPIC, LOWSTATE_ROBOT_TOPIC);

    int cmd_pipe[2];
    int state_pipe[2];
    if (pipe(cmd_pipe) != 0 || pipe(state_pipe) != 0) {
        fprintf(stderr, "Failed to create relay pipes\n");
        return 1;
    }

    pid_t ingress_pid = fork();
    if (ingress_pid < 0) {
        fprintf(stderr, "Failed to fork ingress process\n");
        close(cmd_pipe[PIPE_READ_END]);   close(cmd_pipe[PIPE_WRITE_END]);
        close(state_pipe[PIPE_READ_END]); close(state_pipe[PIPE_WRITE_END]);
        return 1;
    }

    if (ingress_pid == 0) {
        close(cmd_pipe[PIPE_READ_END]);
        close(state_pipe[PIPE_READ_END]);
        int rc = run_ingress(robot_interface,
                             ingress_uses_existing_env,
                             cmd_pipe[PIPE_WRITE_END],
                             state_pipe[PIPE_WRITE_END]);
        _exit(rc);
    }

    close(cmd_pipe[PIPE_WRITE_END]);
    close(state_pipe[PIPE_WRITE_END]);
    int egress_rc = run_egress(sim_interface,
                               cmd_pipe[PIPE_READ_END],
                               state_pipe[PIPE_READ_END]);

    g_running = false;
    kill(ingress_pid, SIGTERM);

    int ingress_status = 0;
    waitpid(ingress_pid, &ingress_status, 0);

    if (egress_rc != 0) return egress_rc;
    if (!WIFEXITED(ingress_status) || WEXITSTATUS(ingress_status) != 0) return 1;
    return 0;
}
