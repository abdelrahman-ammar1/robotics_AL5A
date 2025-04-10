#include <Arduino.h>
#include "BasicLinearAlgebra.h"

using namespace BLA;

#define NUM_JOINTS 4
#define DELTA 0.001f
#define STEP_SIZE 0.1f
#define JOINT_RANGE_OF_MOTION 3.14159

float d1 = 6.9;             // Base height
float a2 = 9.4, a3 = 10.8;  // Link lengths >> (approx: 3.75 inch and 4.25 inch)
float a4 = 8.5725;          // Gripper to wrist >> (approx: 3.375 inch)

// Forward kinematics function
Matrix<3,1> forwardKinematics(Matrix<NUM_JOINTS,1> q) {
    Matrix<3,1> e;
    e(0) = a4 * cos(q(0)) * cos(q(1) + q(2) + q(3))   +   a3 * cos(q(0)) * cos(q(1) + q(2))   +   a2 * cos(q(0)) * cos(q(1));   // X position
    e(1) = a4 * sin(q(0)) * sin(q(1) + q(2) + q(3))   +   a3 * sin(q(0)) * cos(q(1) + q(2))   +   a2 * sin(q(0)) * cos(q(1));   // Y position
    e(2) = -1 * a4 * sin(q(1) + q(2) + q(3))   -   a3 * sin(q(1) + q(2))  -   a2 * sin(q(1))  +   d1;                           // Z position
    return e;
}

// Compute numerical Jacobian
Matrix<3, NUM_JOINTS> computeJacobian(Matrix<NUM_JOINTS,1> q) {
    Matrix<3, NUM_JOINTS> J;
    Matrix<3,1> e = forwardKinematics(q); // Compute initial end-effector position

    for (int i = 0; i < NUM_JOINTS; i++) {
        Matrix<NUM_JOINTS,1> q_perturbed = q;
        q_perturbed(i) += DELTA; // Apply small perturbation
        Matrix<3,1> e_perturbed = forwardKinematics(q_perturbed);

        // Compute numerical derivative for each row
        for (int row = 0; row < 3; row++) {
            J(row, i) = (e_perturbed(row) - e(row)) / DELTA;
        }
    }
    return J;
}

// Compute damped least squares pseudoinverse of the Jacobian
Matrix<NUM_JOINTS,3> computeDampedPseudoinverse(Matrix<3, NUM_JOINTS> J, float lambda) {
    Matrix<3,3> JJT = J * (~J); // Compute J * J^T
    Matrix<3,3> I = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // Identity matrix

    for (int i = 0; i < 3; i++) {
        I(i, i) *= lambda * lambda; // Scale identity by lambda^2
    }

    Matrix<3,3> JJT_damped = JJT + I; // Add damping
    Invert(JJT_damped); // Compute inverse

    return (~J) * JJT_damped; // Compute J^T * (JJT_damped)^-1
}

void setup() {
    Serial.begin(9600);

    float damping_factor = 0.1;

    // Set initial joint angles and target end-effector position
    Matrix<NUM_JOINTS,1> q = {0.0, -1.5708, -1.2, 0.0}; // Initial joint angles (radians)
    Matrix<3,1> g = {20.0, 0.0, 10.0}; // Desired end-effector position
    int iter = 0;
    Matrix<3,1> e = forwardKinematics(q); // Compute initial end-effector position

    Serial.println("Starting Inverse Kinematics Computation...\n");

    while ((~(e-g)*(e-g))(0,0) >= 0.000001) { // While error is above threshold
        iter++;

        // Compute Jacobian
        Matrix<3, NUM_JOINTS> J = computeJacobian(q);

        // Compute Damped Pseudoinverse
        Matrix<NUM_JOINTS,3> J_pseudo = computeDampedPseudoinverse(J, damping_factor);

        // Compute error (Δe = g - e)
        Matrix<3,1> delta_e = g - e;
        delta_e *= STEP_SIZE; // Scale step size to prevent overshooting

        // Compute joint angle updates (Δθ = J_pseudo * Δe)
        Matrix<NUM_JOINTS,1> delta_q = J_pseudo * delta_e;

        // Update joint angles
        q += delta_q;

        // Compute updated end-effector position
        e = forwardKinematics(q);

        // Print iteration details
        Serial.print("Iteration "); Serial.print(iter);
        Serial.print(": q = [");
        for (int i = 0; i < NUM_JOINTS; i++) {
            Serial.print(q(i), 4);
            Serial.print(i < NUM_JOINTS - 1 ? ", " : "]  ");
        }

        // Print current end-effector coordinates
        Serial.print("End-Effector Position: [");
        Serial.print(e(0), 4); Serial.print(", ");
        Serial.print(e(1), 4); Serial.print(", ");
        Serial.print(e(2), 4); Serial.println("]");
        
        // Stop if maximum iterations exceeded
        if (iter > 1000) {
            Serial.println("Max iterations reached. Exiting.");
            break;
        }
    }

    Serial.println("\nInverse Kinematics Completed.");
    Serial.print("Final Joint Angles: [");
    for (int i = 0; i < NUM_JOINTS; i++) {
        Serial.print(q(i), 4);
        Serial.print(i < NUM_JOINTS - 1 ? ", " : "]\n");
    }

    Serial.print("Final End-Effector Position: [");
    Serial.print(e(0), 4); Serial.print(", ");
    Serial.print(e(1), 4); Serial.print(", ");
    Serial.print(e(2), 4); Serial.println("]\n");

    Serial.println("Goal Position Reached ✅");
}

void loop() {
    // No need for loop actions
}
