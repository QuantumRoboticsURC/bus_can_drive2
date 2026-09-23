// test_percent_output.cpp
// Prueba de simetría en lazo abierto para un Talon SRX.
// Manda pulsos cortos en PercentOutput con magnitudes iguales en ambas direcciones
// y mide cuántos ticks se movió el encoder en cada uno. Si el motor responde
// simétrico, el problema NO es eléctrico ni mecánico — es del PID/MotionMagic.

#define Phoenix_No_WPI

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <vector>
#include <csignal>
#include <atomic>

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

// ═══════════════════════════════════════════════════════════════════════════
// CONFIGURACIÓN — cambia esto según lo que quieras probar
// ═══════════════════════════════════════════════════════════════════════════
constexpr int    CAN_ID        = 14;     // ID del Talon a probar
constexpr double PULSE_SEC     = 0.6;    // duración de cada pulso (segundos)
constexpr int    PAUSE_MS      = 600;    // pausa entre pulsos (ms)
// Alterna direcciones para no derivar lejos del centro
const std::vector<double> TEST_PCT = {
    +0.10, -0.10,
    +0.15, -0.15,
    +0.20, -0.20,
    +0.25, -0.25,
};
// ═══════════════════════════════════════════════════════════════════════════

std::atomic<bool> abort_now{false};
TalonSRX* g_motor = nullptr;

void sigint_handler(int) {
    abort_now.store(true);
}

void neutralize() {
    if (g_motor) g_motor->Set(ControlMode::PercentOutput, 0.0);
}

struct PulseResult {
    int start_ticks;
    int end_ticks;
    int delta;
};

// Maneja el motor por `seconds` con `pct`, refrescando FeedEnable y
// muestreando el encoder. Devuelve ticks de inicio, fin y delta.
PulseResult drive_for(TalonSRX& m, double pct, double seconds) {
    PulseResult r;
    r.start_ticks = m.GetSelectedSensorPosition();

    auto t_end = std::chrono::steady_clock::now()
                 + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < t_end) {
        if (abort_now.load()) break;
        unmanaged::FeedEnable(100);
        m.Set(ControlMode::PercentOutput, pct);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Frena y deja asentar el encoder antes de leer fin
    m.Set(ControlMode::PercentOutput, 0.0);
    unmanaged::FeedEnable(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    r.end_ticks = m.GetSelectedSensorPosition();
    r.delta     = r.end_ticks - r.start_ticks;
    return r;
}

int main() {
    std::signal(SIGINT, sigint_handler);

    std::string iface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(iface.c_str());

    TalonSRX motor(CAN_ID);
    g_motor = &motor;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    int fw = motor.GetFirmwareVersion();
    if (fw == -1) {
        std::cerr << "❌ Talon ID " << CAN_ID << " no responde en el bus CAN\n";
        return 1;
    }
    std::cout << "✅ Talon ID " << CAN_ID << " firmware: " << fw << "\n\n";

    // Misma config básica que en initAll para srxArm5
    motor.ConfigFactoryDefault();
    motor.SetInverted(true);
    motor.SetSensorPhase(false);
    motor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
    motor.ConfigNominalOutputForward(0, 10);
    motor.ConfigNominalOutputReverse(0, 10);
    motor.ConfigPeakOutputForward(1, 10);
    motor.ConfigPeakOutputReverse(-1, 10);

    std::cout << "═══════════════════════════════════════════════════════════\n";
    std::cout << " TEST DE SIMETRÍA EN LAZO ABIERTO (PercentOutput)\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";
    std::cout << " CAN ID:    " << CAN_ID << "\n";
    std::cout << " Pulso:     " << PULSE_SEC << " s\n";
    std::cout << " Pausa:     " << PAUSE_MS << " ms entre pulsos\n\n";
    std::cout << " ⚠️  Coloca MANUALMENTE el joint cerca del centro de su rango\n";
    std::cout << "     antes de empezar, para no chocar con los topes.\n";
    std::cout << " ⚠️  Mantén la mano cerca del paro de emergencia.\n\n";
    std::cout << " Presiona ENTER para empezar (Ctrl+C aborta)...";
    std::cin.get();
    std::cout << "\n";

    int initial = motor.GetSelectedSensorPosition();
    std::cout << " Posición inicial: " << initial << " ticks\n\n";

    std::vector<std::pair<double, PulseResult>> results;

    for (double pct : TEST_PCT) {
        if (abort_now.load()) {
            std::cout << "\n⚠️  Abortado por usuario.\n";
            break;
        }

        std::cout << "── PercentOutput = "
                  << std::showpos << std::fixed << std::setprecision(2)
                  << pct << std::noshowpos << " ──\n";

        PulseResult r = drive_for(motor, pct, PULSE_SEC);
        results.push_back({pct, r});

        double tps = r.delta / PULSE_SEC;
        std::cout << "   start=" << r.start_ticks
                  << "  end=" << r.end_ticks
                  << "  Δ=" << std::showpos << r.delta << std::noshowpos
                  << "   (" << std::fixed << std::setprecision(1)
                  << tps << " ticks/s)\n\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(PAUSE_MS));
    }

    neutralize();
    unmanaged::FeedEnable(100);

    // ─── Resumen ───
    std::cout << "═══════════════════════════════════════════════════════════\n";
    std::cout << " RESUMEN\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";
    std::cout << std::setw(10) << "Pct"
              << std::setw(12) << "Delta"
              << std::setw(14) << "ticks/seg"
              << "\n";
    std::cout << "  ────────────────────────────────────\n";
    for (auto& p : results) {
        double tps = p.second.delta / PULSE_SEC;
        std::cout << std::setw(10) << std::fixed << std::setprecision(2) << p.first
                  << std::setw(12) << std::showpos << p.second.delta << std::noshowpos
                  << std::setw(14) << std::fixed << std::setprecision(1) << tps
                  << "\n";
    }

    std::cout << "\n💡 Cómo leer esto:\n";
    std::cout << "   - Para cada par (+pct, -pct) la magnitud de Δ debería ser parecida.\n";
    std::cout << "     Ej. +0.20 mueve +850 y -0.20 mueve -820 → simétrico.\n";
    std::cout << "   - Si +0.20 mueve +900 y -0.20 solo mueve -100 (o nada) →\n";
    std::cout << "     ahí está la asimetría. Causas probables:\n";
    std::cout << "       * gravedad/fricción mecánica en ese sentido\n";
    std::cout << "       * cableado del motor (M+/M-) que ya no corresponde\n";
    std::cout << "       * un MOSFET de la H-bridge dañado (si swappeaste Talon\n";
    std::cout << "         y persiste, descartado)\n";
    std::cout << "   - Si la simetría es buena en lazo abierto pero falla en\n";
    std::cout << "     MotionMagic → el problema es del PID (NominalOutput,\n";
    std::cout << "     integral windup, aceleración).\n";

    return 0;
}
