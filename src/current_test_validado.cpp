// Reproducir desde la Jetson lo que se hacia en Phoenix Tuner:
// mandar un setpoint de corriente al Talon SRX y verificar que el motor entregue esa corriente.
//
// NO se llama ConfigFactoryDefault ni se escriben gains: se respeta lo que ya quedo
// guardado en el Talon desde Tuner (gains, limites de corriente, inversion).
//
// Uso:
//   ./current_test_v3 dump  <can_id>                              Imprime la config guardada en el Talon y sale
//   ./current_test_v3 hold  <can_id> <amps> <secs> [slot] [kP]    Mantiene un setpoint de CORRIENTE constante (amperes)
//   ./current_test_v3 pct   <can_id> <frac> <secs>                Salida en PORCENTAJE (0.2 = 20%), sin lazo de corriente
//   ./current_test_v3 sweep <can_id> [slot]                       Escalones 0.1, 0.2, 0.5, 1.0 A con pausas en 0 A
//
// CSV por stdout:
//   t_s, setpoint_A, stator_A, supply_A, output_A, motor_output_pct, bus_V
// Faults por stderr cada 500 ms (redirige con 2> faults.log).
//
// Dos threads auxiliares:
//   enable_thread: renueva FeedEnable cada 20 ms (evita que el Talon se deshabilite por watchdog)
//   fault_thread:  lee faults del Talon cada 500 ms para diagnosticar brownouts, resets, etc.

#define Phoenix_No_WPI

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
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

// ===================== PARAMETROS =====================

constexpr double MAX_SETPOINT_A = 2.0;
constexpr double LOOP_HZ = 100.0;

const std::vector<double> SWEEP_LEVELS_A = {0.1, 0.2, 0.5, 1.0};
constexpr double SWEEP_HOLD_S  = 2.0;
constexpr double SWEEP_PAUSE_S = 1.0;

// ======================================================

static std::atomic<bool> g_run{true};
static std::atomic<bool> g_enabled{false};

static void on_sigint(int) { g_run = false; }

// Mantiene al Talon habilitado mientras g_enabled sea true.
// Renueva FeedEnable cada 20 ms con una ventana de 100 ms.
static void enable_thread() {
    while (g_run) {
        if (g_enabled) {
            ctre::phoenix::unmanaged::FeedEnable(100);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

// Imprime los faults del Talon cada 500 ms en stderr para diagnosticar
// por que se deshabilita (UnderVoltage, HardwareFailure, ResetDuringEn, etc.).
static void fault_thread(TalonSRX* talon) {
    while (g_run) {
        Faults f;
        talon->GetFaults(f);
        StickyFaults sf;
        talon->GetStickyFaults(sf);
        std::cerr << "faults: UV=" << f.UnderVoltage
                  << " HW=" << f.HardwareFailure
                  << " RstEn=" << f.ResetDuringEn
                  << " APIErr=" << f.APIError
                  << " | sticky UV=" << sf.UnderVoltage
                  << " RstEn=" << sf.ResetDuringEn << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

static double now_s(Clock::time_point t0) {
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

static void dump_config(TalonSRX& talon) {
    TalonSRXConfiguration cfg;
    talon.GetAllConfigs(cfg, 50);
    std::cerr << "Config guardada en el Talon:\n"
              << "  slot0: kP=" << cfg.slot0.kP << " kI=" << cfg.slot0.kI
              << " kD=" << cfg.slot0.kD << " kF=" << cfg.slot0.kF << "\n"
              << "  slot1: kP=" << cfg.slot1.kP << " kI=" << cfg.slot1.kI
              << " kD=" << cfg.slot1.kD << " kF=" << cfg.slot1.kF << "\n"
              << "  slot2: kP=" << cfg.slot2.kP << " kI=" << cfg.slot2.kI
              << " kD=" << cfg.slot2.kD << " kF=" << cfg.slot2.kF << "\n"
              << "  slot3: kP=" << cfg.slot3.kP << " kI=" << cfg.slot3.kI
              << " kD=" << cfg.slot3.kD << " kF=" << cfg.slot3.kF << "\n"
              << "  peakCurrentLimit=" << cfg.peakCurrentLimit
              << " peakCurrentDuration=" << cfg.peakCurrentDuration
              << " continuousCurrentLimit=" << cfg.continuousCurrentLimit << "\n";
}

static bool setpoint_at(const std::string& mode, double t, double hold_a, double hold_s, double& amps) {
    if (mode == "hold" || mode == "pct") {
        if (t >= hold_s) return false;
        amps = hold_a;
        return true;
    }
    const double block = SWEEP_PAUSE_S + SWEEP_HOLD_S;
    const double total = block * SWEEP_LEVELS_A.size() + SWEEP_PAUSE_S;
    if (t >= total) return false;
    const size_t idx = static_cast<size_t>(t / block);
    if (idx >= SWEEP_LEVELS_A.size()) { amps = 0.0; return true; }
    const double in_block = t - idx * block;
    amps = (in_block < SWEEP_PAUSE_S) ? 0.0 : SWEEP_LEVELS_A[idx];
    return true;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Uso: " << argv[0] << " dump <can_id>\n"
                  << "     " << argv[0] << " hold <can_id> <amps> <secs> [slot] [kP]\n"
                  << "     " << argv[0] << " pct <can_id> <frac> <secs>\n"
                  << "     " << argv[0] << " sweep <can_id> [slot]\n";
        return 1;
    }
    const std::string mode = argv[1];
    const int can_id = std::stoi(argv[2]);

    double hold_a = 0.0, hold_s = 0.0;
    int slot = 0;
    bool override_kp = false;
    double kp_override = 0.0;

    if (mode == "hold" || mode == "pct") {
        if (argc < 5) { std::cerr << mode << " necesita <valor> <secs>\n"; return 1; }
        hold_a = std::stod(argv[3]);
        hold_s = std::stod(argv[4]);
        if (argc >= 6) slot = std::stoi(argv[5]);
        if (argc >= 7) { override_kp = true; kp_override = std::stod(argv[6]); }
        if (std::fabs(hold_a) > MAX_SETPOINT_A) {
            std::cerr << "Setpoint " << hold_a << " A excede el tope de software (" << MAX_SETPOINT_A << " A)\n";
            return 1;
        }
    } else if (mode == "sweep") {
        if (argc >= 4) slot = std::stoi(argv[3]);
    } else if (mode != "dump") {
        std::cerr << "Modo invalido: " << mode << "\n";
        return 1;
    }

    std::signal(SIGINT, on_sigint);
    std::signal(SIGTERM, on_sigint);

    ctre::phoenix::platform::can::SetCANInterface("can0");
    TalonSRX talon(can_id);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (talon.GetFirmwareVersion() == -1) {
        std::cerr << "El Talon ID " << can_id << " no responde en can0\n";
        return 1;
    }

    dump_config(talon);
    if (mode == "dump") return 0;

    // Limpiar sticky faults de corridas pasadas, para que lo que veamos sea de esta prueba
    talon.ClearStickyFaults(50);

    talon.SelectProfileSlot(slot, 0);
    std::cerr << "Usando slot " << slot << ". Setpoints en AMPERES segun la documentacion de Phoenix 5.\n";

    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 10);

    if (override_kp) {
        talon.Config_kP(slot, kp_override, 50);
        std::cerr << "kP del slot " << slot << " puesto en " << kp_override << " para esta prueba\n";
    }

    // Arrancar los threads auxiliares
    g_enabled = true;
    std::thread enabler(enable_thread);
    std::thread faulter(fault_thread, &talon);

    // Calentamiento: 3 s a 0, para que la libreria termine de iniciar
    for (int i = 0; i < 300 && g_run; ++i) {
        if (mode == "pct") talon.Set(ControlMode::PercentOutput, 0.0);
        else               talon.Set(ControlMode::Current, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const auto period = std::chrono::duration<double>(1.0 / LOOP_HZ);
    const auto t0 = Clock::now();
    auto next = t0;

    std::cout << "t_s,setpoint_A,stator_A,supply_A,output_A,motor_output_pct,bus_V\n";

    while (g_run) {
        const double t = now_s(t0);
        double amps = 0.0;
        if (!setpoint_at(mode, t, hold_a, hold_s, amps)) break;

        if (mode == "pct") talon.Set(ControlMode::PercentOutput, amps);
        else               talon.Set(ControlMode::Current, amps);

        std::cout << t << "," << amps << ","
                  << talon.GetStatorCurrent() << "," << talon.GetSupplyCurrent() << ","
                  << talon.GetOutputCurrent() << ","
                  << talon.GetMotorOutputPercent() << "," << talon.GetBusVoltage() << "\n";

        next += std::chrono::duration_cast<Clock::duration>(period);
        std::this_thread::sleep_until(next);
    }

    // Apagado seguro: baja a 0 A manteniendo el enable, luego corta el enable
    for (int i = 0; i < 20; ++i) {
        talon.Set(ControlMode::Current, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    talon.Set(ControlMode::PercentOutput, 0.0);

    g_enabled = false;
    g_run = false;
    enabler.join();
    faulter.join();

    std::cerr << "Listo, motor en 0 A.\n";
    return 0;
}
