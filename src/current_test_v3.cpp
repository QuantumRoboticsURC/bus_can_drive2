// Prueba de control por corriente de un Talon SRX desde la Jetson (Phoenix 5).
// Manda un setpoint de corriente (o porcentaje) y registra en CSV lo que entrega
// el Talon y lo que mide su encoder.
//
// Uso:
//   current_test_v3 dump  <can_id>
//   current_test_v3 hold  <can_id> <amps> <secs> [slot] [kP]
//   current_test_v3 pct   <can_id> <frac> <secs>
//   current_test_v3 sweep <can_id> [slot]
//
// stdout: CSV con t_s, setpoint_A, stator_A, supply_A, output_A, motor_output_pct,
//         bus_V, pos_ticks, pos_turns, vel_turns_s
// stderr: banner inicial, faults cada 500 ms, resumen final
//
// NO se tocan los gains ni los limites de corriente del Talon (se respeta Tuner),
// excepto cuando se pasa [kP]. SI se configura el sensor (Mag Encoder absoluto, 4096 cpr)
// porque es necesario para leer posicion y velocidad.

#define Phoenix_No_WPI

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using Clock = std::chrono::steady_clock;

// ============================= PARAMETROS =============================

constexpr double MAX_SETPOINT_A  = 9.2;
constexpr double LOOP_HZ         = 100.0;
constexpr double COUNTS_PER_REV  = 4096.0;  // Mag Encoder absoluto, 12 bits
constexpr double WARMUP_S        = 3.0;     // 0 A antes del setpoint real

const std::vector<double> SWEEP_LEVELS_A = {0.1, 0.2, 0.5, 1.0};
constexpr double SWEEP_HOLD_S  = 2.0;
constexpr double SWEEP_PAUSE_S = 1.0;

// ============================= ESTADO GLOBAL ==========================

static std::atomic<bool> g_run{true};
static std::atomic<bool> g_enabled{false};

static void on_sigint(int) { g_run = false; }

// ============================= CONFIG Y ARGS ==========================

struct Config {
    std::string mode;
    int    can_id       = 0;
    double hold_a       = 0.0;
    double hold_s       = 0.0;
    int    slot         = 0;
    bool   override_kp  = false;
    double kp_override  = 0.0;
};

static int parse_args(int argc, char** argv, Config& c) {
    if (argc < 3) {
        std::cerr << "Uso:\n"
                  << "  " << argv[0] << " dump <can_id>\n"
                  << "  " << argv[0] << " hold <can_id> <amps> <secs> [slot] [kP]\n"
                  << "  " << argv[0] << " pct <can_id> <frac> <secs>\n"
                  << "  " << argv[0] << " sweep <can_id> [slot]\n";
        return 1;
    }
    c.mode   = argv[1];
    c.can_id = std::stoi(argv[2]);

    if (c.mode == "dump") return 0;

    if (c.mode == "hold" || c.mode == "pct") {
        if (argc < 5) { std::cerr << "ERR: " << c.mode << " necesita <valor> <secs>\n"; return 1; }
        c.hold_a = std::stod(argv[3]);
        c.hold_s = std::stod(argv[4]);
        if (argc >= 6) c.slot = std::stoi(argv[5]);
        if (argc >= 7) { c.override_kp = true; c.kp_override = std::stod(argv[6]); }
        if (std::fabs(c.hold_a) > MAX_SETPOINT_A) {
            std::cerr << "ERR: setpoint " << c.hold_a
                      << " excede el tope de software (" << MAX_SETPOINT_A << ")\n";
            return 1;
        }
    } else if (c.mode == "sweep") {
        if (argc >= 4) c.slot = std::stoi(argv[3]);
    } else {
        std::cerr << "ERR: modo invalido: " << c.mode << "\n";
        return 1;
    }
    return 0;
}

// ============================= TALON HELPERS ==========================

static int connect_talon(TalonSRX& talon) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    const int fw = talon.GetFirmwareVersion();
    if (fw == -1) {
        std::cerr << "ERR: Talon no responde en can0\n";
        return 1;
    }
    return 0;
}

static void dump_config(TalonSRX& talon) {
    TalonSRXConfiguration cfg;
    talon.GetAllConfigs(cfg, 50);
    std::cerr << "slot0: kP=" << cfg.slot0.kP << " kI=" << cfg.slot0.kI
              << " kD=" << cfg.slot0.kD << " kF=" << cfg.slot0.kF << "\n"
              << "slot1: kP=" << cfg.slot1.kP << " kI=" << cfg.slot1.kI
              << " kD=" << cfg.slot1.kD << " kF=" << cfg.slot1.kF << "\n"
              << "slot2: kP=" << cfg.slot2.kP << " kI=" << cfg.slot2.kI
              << " kD=" << cfg.slot2.kD << " kF=" << cfg.slot2.kF << "\n"
              << "slot3: kP=" << cfg.slot3.kP << " kI=" << cfg.slot3.kI
              << " kD=" << cfg.slot3.kD << " kF=" << cfg.slot3.kF << "\n"
              << "limits: peak=" << cfg.peakCurrentLimit
              << "A dur=" << cfg.peakCurrentDuration
              << "ms cont=" << cfg.continuousCurrentLimit << "A\n";
}

static void setup_sensor(TalonSRX& talon) {
    talon.ClearStickyFaults(50);
    talon.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
    talon.SetSensorPhase(false);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 10);
}

// ============================= THREADS AUXILIARES =====================

static void enable_thread() {
    while (g_run) {
        if (g_enabled) ctre::phoenix::unmanaged::FeedEnable(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

static void fault_thread(TalonSRX* talon) {
    while (g_run) {
        Faults f;        talon->GetFaults(f);
        StickyFaults sf; talon->GetStickyFaults(sf);
        std::cerr << "faults: UV=" << f.UnderVoltage
                  << " HW=" << f.HardwareFailure
                  << " RstEn=" << f.ResetDuringEn
                  << " APIErr=" << f.APIError
                  << " | sticky UV=" << sf.UnderVoltage
                  << " RstEn=" << sf.ResetDuringEn << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

// ============================= GENERACION DE SETPOINT =================

// Devuelve false cuando la rutina terminó.
static bool setpoint_at(const Config& c, double t, double& amps) {
    if (c.mode == "hold" || c.mode == "pct") {
        if (t >= c.hold_s) return false;
        amps = c.hold_a;
        return true;
    }
    // sweep
    const double block = SWEEP_PAUSE_S + SWEEP_HOLD_S;
    const double total = block * SWEEP_LEVELS_A.size() + SWEEP_PAUSE_S;
    if (t >= total) return false;
    const size_t idx = static_cast<size_t>(t / block);
    if (idx >= SWEEP_LEVELS_A.size()) { amps = 0.0; return true; }
    const double in_block = t - idx * block;
    amps = (in_block < SWEEP_PAUSE_S) ? 0.0 : SWEEP_LEVELS_A[idx];
    return true;
}

// ============================= LAZO PRINCIPAL =========================

struct Stats {
    long   samples      = 0;
    double duration_s   = 0.0;
    double pos_turns_i  = 0.0;
    double pos_turns_f  = 0.0;
    double vel_mean     = 0.0;
    double stator_mean  = 0.0;
};

static void send_setpoint(TalonSRX& talon, const std::string& mode, double v) {
    if (mode == "pct") talon.Set(ControlMode::PercentOutput, v);
    else               talon.Set(ControlMode::Current, v);
}

static void warmup(TalonSRX& talon, const std::string& mode) {
    const int ticks = static_cast<int>(WARMUP_S * 100);
    for (int i = 0; i < ticks && g_run; ++i) {
        send_setpoint(talon, mode, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

static Stats run_loop(TalonSRX& talon, const Config& c) {
    Stats s;
    s.pos_turns_i = talon.GetSelectedSensorPosition() / COUNTS_PER_REV;

    std::cout << "t_s,setpoint_A,stator_A,supply_A,output_A,motor_output_pct,bus_V,"
                 "pos_ticks,pos_turns,vel_turns_s\n";

    const auto period = std::chrono::duration<double>(1.0 / LOOP_HZ);
    const auto t0 = Clock::now();
    auto next = t0;

    double sum_vel = 0.0, sum_stator = 0.0;

    while (g_run) {
        const double t = std::chrono::duration<double>(Clock::now() - t0).count();
        double amps = 0.0;
        if (!setpoint_at(c, t, amps)) break;

        send_setpoint(talon, c.mode, amps);

        const double pos_ticks   = talon.GetSelectedSensorPosition();
        const double pos_turns   = pos_ticks / COUNTS_PER_REV;
        const double vel_turns_s = talon.GetSelectedSensorVelocity() * 10.0 / COUNTS_PER_REV;
        const double stator      = talon.GetStatorCurrent();

        std::cout << t << "," << amps << ","
                  << stator << "," << talon.GetSupplyCurrent() << ","
                  << talon.GetOutputCurrent() << ","
                  << talon.GetMotorOutputPercent() << "," << talon.GetBusVoltage() << ","
                  << pos_ticks << "," << pos_turns << "," << vel_turns_s << "\n";

        sum_vel    += vel_turns_s;
        sum_stator += stator;
        s.samples  += 1;
        s.duration_s = t;
        s.pos_turns_f = pos_turns;

        next += std::chrono::duration_cast<Clock::duration>(period);
        std::this_thread::sleep_until(next);
    }

    if (s.samples > 0) {
        s.vel_mean    = sum_vel    / s.samples;
        s.stator_mean = sum_stator / s.samples;
    }
    return s;
}

static void stop_motor(TalonSRX& talon) {
    for (int i = 0; i < 20; ++i) {
        talon.Set(ControlMode::Current, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    talon.Set(ControlMode::PercentOutput, 0.0);
}

// ============================= BANNER Y RESUMEN =======================

static void print_banner(const Config& c) {
    std::cerr << "=== current_test_v3 ===\n"
              << "mode:     " << c.mode << "\n"
              << "can_id:   " << c.can_id << "\n";
    if (c.mode == "hold" || c.mode == "pct") {
        std::cerr << "setpoint: " << c.hold_a << (c.mode == "pct" ? " (pct)" : " A") << "\n"
                  << "duration: " << c.hold_s << " s + " << WARMUP_S << " s de warmup\n"
                  << "slot:     " << c.slot << "\n";
        if (c.override_kp)
            std::cerr << "kP:       " << c.kp_override << " (forzado en el slot)\n";
    } else if (c.mode == "sweep") {
        std::cerr << "sweep:    0.1, 0.2, 0.5, 1.0 A (" << SWEEP_HOLD_S
                  << "s on, " << SWEEP_PAUSE_S << "s off)\n"
                  << "slot:     " << c.slot << "\n";
    }
    std::cerr << "loop:     " << LOOP_HZ << " Hz\n"
              << "========================\n";
}

static void print_summary(const Stats& s) {
    const double delta_turns = s.pos_turns_f - s.pos_turns_i;
    std::cerr << std::fixed << std::setprecision(3)
              << "=== resumen ===\n"
              << "samples:       " << s.samples << "\n"
              << "duration:      " << s.duration_s << " s\n"
              << "pos_inicial:   " << s.pos_turns_i << " turns\n"
              << "pos_final:     " << s.pos_turns_f << " turns\n"
              << "delta:         " << delta_turns << " turns\n"
              << "vel_promedio:  " << s.vel_mean << " turns/s\n"
              << "stator_prom:   " << s.stator_mean << " A\n"
              << "===============\n";
}

// ============================= MAIN ==================================

int main(int argc, char** argv) {
    Config c;
    if (parse_args(argc, argv, c) != 0) return 1;

    std::signal(SIGINT, on_sigint);
    std::signal(SIGTERM, on_sigint);

    ctre::phoenix::platform::can::SetCANInterface("can0");
    TalonSRX talon(c.can_id);

    if (connect_talon(talon) != 0) return 1;

    if (c.mode == "dump") {
        dump_config(talon);
        return 0;
    }

    print_banner(c);
    dump_config(talon);

    setup_sensor(talon);
    talon.SelectProfileSlot(c.slot, 0);
    if (c.override_kp) talon.Config_kP(c.slot, c.kp_override, 50);

    g_enabled = true;
    std::thread enabler(enable_thread);
    std::thread faulter(fault_thread, &talon);

    warmup(talon, c.mode);
    Stats s = run_loop(talon, c);
    stop_motor(talon);

    g_enabled = false;
    g_run = false;
    enabler.join();
    faulter.join();

    print_summary(s);
    return 0;
}
