#include <iostream>
#include <cmath>

// Curva di spinta approssimata più vicina alla realtà (Durata ~0.2s, Impulso ~2 Ns)
double getThrust(double t) {
    if (t <= 0.20) {
        // Decadimento lineare da 12.36 N a 0 N nei primi 0.2 secondi
        return 12.36* (1.0 - (t / 0.20));
    }
    return 0.0;
}

double simulateTime(double Cd, double m, double rho, double A, double target_x, double dt, double mu) {
    double x = 0.0, v = 0.0, t = 0.0;
    double g = 9.81;
    double f_roll = mu * m * g; // Attrito di rotolamento costante

    while (x < target_x) {
        double thrust = getThrust(t);
        double drag = 0.5 * rho * A * Cd * v * v;
        
        // Forza netta = Spinta - Drag Aerodinamico - Attrito Meccanico
        double total_drag = drag + f_roll;
        double a = (thrust - total_drag) / m;
        
        // Se la vettura esaurisce la spinta e le resistenze la fermano prima dei 20m
        if (v <= 0 && thrust <= 0) return 999.0;

        v += a * dt;
        if (v < 0) v = 0; // Impedisce alla macchina di andare a marcia indietro per attrito

        x += v * dt;
        t += dt;

        if (t > 5.0) return 999.0;
    }
    return t;
}

int main() {
    // Dati
    double m = 0.04687;        // 46.87 g in kg
    double A = 0.002166;       // 0.002166 m^2
    double target_time = 0.9;  // Target 0.9s
    double target_x = 20.0;    // 20 metri
    double rho = 1.225;
    double dt = 0.0001;        // Passo più fine (0.1 ms) per gestire picchi veloci
    double mu = 0.15;          // Coeff. attrito rotolamento/cuscinetti fittizio

    double cd_min = 0.001;
    double cd_max = 2.0;
    double cd_mid = 0.0;
    double tolerance = 0.0001;

    while ((cd_max - cd_min) > tolerance) {
        cd_mid = (cd_min + cd_max) / 2.0;
        double current_time = simulateTime(cd_mid, m, rho, A, target_x, dt, mu);

        if (current_time <= target_time) {
            cd_min = cd_mid;
        } else {
            cd_max = cd_mid;
        }
    }

    std::cout << "=== RISULTATI CON SPINTA IMPULSIVA REALE ===" << std::endl;
    std::cout << "CD massimo consentito per 1.0s: " << cd_mid << std::endl;
    std::cout << "Tempo con questo CD: " << simulateTime(cd_mid, m, rho, A, target_x, dt, mu) << " s" << std::endl;

    // Verifica del tempo attuale reale (es. assumendo un CD tipico di 0.25)
    std::cout << "Tempo stimato con CD = 0.25: " << simulateTime(0.15, m, rho, A, target_x, dt, mu) << " s" << std::endl;

    return 0;
}

