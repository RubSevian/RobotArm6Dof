#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include "src/communication/serial_port.h"
#include "src/drivers/motor_control.h"


static void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static double now_s() {
    using clock = std::chrono::steady_clock;
    static const auto t0 = clock::now();
    auto t = clock::now() - t0;
    return std::chrono::duration<double>(t).count();
}

int main() {
    try {
        serialport::SerialPortWrapper serial;
        serial.startReceiving();

        MotorControl mc(serial);

        float pos[6] = {0}, vel[6] = {0}, kp[6] = {0}, kd[6] = {0}, tau[6] = {0};

        // Список реально используемых моторов (по твоему маппингу)
        const int motors[] = {0, 1};
        constexpr int N = sizeof(motors) / sizeof(motors[0]);

        // Безопасные параметры
        for (int i = 0; i < N; ++i) {
            int m = motors[i];
            kp[m] = 0.8f;
            kd[m] = 0.4f;
            vel[m] = 0.0f;
            tau[m] = 0.0f;
            pos[m] = 0.0f;
        }

        std::cout << "ENABLE...\n";
        mc.EnableMotors(serial);
        sleep_ms(300);
        mc.EnableMotors(serial);
        sleep_ms(700);
        const float A = 60.f;      // амплитуда (рад) 
        const float f = 0.2f;      // частота (Гц) 
        const float w = 2.0f * 3.1415926535f * f;

        const int period_ms = 1;   
        const double T = 100.0;     // 10 секунд синус

        std::cout << "SINE for " << T << " s ...\n";

        double t_start = now_s();
        while (now_s() - t_start < T) {
            float s = A * std::sin(w * (float)(now_s() - t_start));

            // Например: мотор1 = +sin, мотор2 = -sin
            pos[motors[0]] =  s;
            pos[motors[1]] = -s;

            mc.ControlMotors(serial, pos, vel, kp, kd, tau);
            sleep_ms(period_ms);
        }

        // ---- Корректная остановка перед disable ----
        std::cout << "STOP: send zeros...\n";
        for (int i = 0; i < N; ++i) {
            int m = motors[i];
            pos[m] = 0.0f;
            vel[m] = 0.0f;
            tau[m] = 0.0f;

        }

        for (int k = 0; k < 100; ++k) {
            mc.ControlMotors(serial, pos, vel, kp, kd, tau);
            sleep_ms(period_ms);
        }

        // Disable несколько раз (как enable)
        // --- DISABLE ---
        std::cout << "DISABLE...\n";
        mc.DisableMotors(serial);

        serial.stopReceiving();
        std::cout << "DONE.\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "ERR: " << e.what() << "\n";
        return 1;
    }
}

    //     // --- Serial ---
    //     serialport::SerialPortWrapper serial;
    //     serial.startReceiving();

    //     MotorControl mc(serial);

    //     // --- Протокольные массивы на 6 моторов (используем выбранные индексы) ---
    //     float pos[6] = {0}, vel[6] = {0}, kp[6] = {0}, kd[6] = {0}, tau[6] = {0};

    //     // ✅ Список моторов, которые реально подключены и должны двигаться
    //     // По твоей проверке первый мотор = индекс 1, второй обычно = 2
    //     const int motors[] = {0, 1};
    //     constexpr int MOTORS_N = sizeof(motors) / sizeof(motors[0]);

    //     // --- Безопасные коэффициенты + нулевые vel/tau ---
    //     for (int i = 0; i < MOTORS_N; ++i) {
    //         int m = motors[i];
    //         kp[m] = 0.8f;
    //         kd[m] = 0.2f;
    //         vel[m] = 0.0f;
    //         tau[m] = 4.0f;
    //         pos[m] = 0.0f;
    //     }

    //     std::cout << "=== DAMIAO MOTOR TEST ===\n";
    //     std::cout << "Motors indices: ";
    //     for (int i = 0; i < MOTORS_N; ++i) std::cout << motors[i] << (i+1<MOTORS_N? ", ":"");
    //     std::cout << "\n";

    //     // --- ENABLE (часто делают два раза) ---
    //     std::cout << "ENABLE...\n";
    //     mc.EnableMotors(serial);
    //     sleep_ms(300);
    //     mc.EnableMotors(serial);
    //     sleep_ms(700);

    //     // ---------- STEP 1: небольшое движение ----------
    //     // Пример: мотор 1 в +0.2, мотор 2 в -0.2
    //     std::cout << "MOVE STEP: +0.2 / -0.2\n";
    //     pos[motors[0]] = 0.9f;
    //     pos[motors[1]] = -0.6f;

    //     tau[motors[1]] = 10;

    //     // Посылаем поток команд ~2 секунды (5ms период -> ~200 Гц)
    //     for (int t = 0; t < 400; ++t) {
    //         mc.ControlMotors(serial, pos, vel, kp, kd, tau);
    //         sleep_ms(5);
    //     }

    //     // ---------- STEP 2: вернуть в ноль ----------
    //     std::cout << "MOVE STEP: back to 0.0\n";
    //     for (int i = 0; i < MOTORS_N; ++i) {
    //         int m = motors[i];
    //         pos[m] = 0.0f;
    //     }

    //     for (int t = 0; t < 400; ++t) {
    //         mc.ControlMotors(serial, pos, vel, kp, kd, tau);
    //         sleep_ms(5);
    //     }

    //     // --- Диагностика: покажем первые байты входных данных от STM (если есть) ---
    //     auto rx = serial.getReceivedData();
    //     std::cout << "RX[0..15]: ";
    //     for (int i = 0; i < 16; ++i) {
    //         std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)rx[i] << " ";
    //     }
    //     std::cout << std::dec << "\n";

    //     // --- DISABLE ---
    //     std::cout << "DISABLE...\n";
    //     mc.DisableMotors(serial);

    //     serial.stopReceiving();
    //     std::cout << "DONE.\n";
    //     return 0;
    // }
    // catch (const std::exception& e) {
    //     std::cerr << "ERR: " << e.what() << "\n";
    //     return 1;
    // }
//}