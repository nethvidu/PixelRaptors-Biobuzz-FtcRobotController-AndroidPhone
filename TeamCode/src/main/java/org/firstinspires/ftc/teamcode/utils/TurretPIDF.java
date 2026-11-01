package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Position PIDF controller for the {@link Turret}. It is time-aware (uses real dt), computes the
 * derivative on the error, clamps the integral for anti-windup, skips I/D on the first loop to
 * avoid a stale-dt spike, applies velocity feedforward on the target rate, and clamps the output
 * to [-1, 1]. Feed it continuous (un-wrapped) angles — it does no angle wrapping.
 */
public class TurretPIDF {

    // --- Tuning Gains ---
    private double kP, kI, kD, kF;

    // --- State Variables ---
    private double integralSum = 0;
    private double lastError = 0;
    private boolean firstLoop = true;
    private double lastTarget = 0;

    // --- Anti-windup limits ---
    private double maxIntegral = 1.0;

    // --- Target ---
    private double target;

    // --- Timer ---
    private final ElapsedTime timer;

    /**
     * Constructor for the PIDF Controller.
     *
     * @param kP Proportional gain.
     * @param kI Integral gain.
     * @param kD Derivative gain.
     * @param kF Feedforward gain.
     */
    public TurretPIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.timer = new ElapsedTime();
    }

    /**
     * Calculates the necessary correction power based on the current state and target.
     * This should be called once per loop.
     *
     * @param current The current measurement of the system (e.g., angle, velocity).
     * @return The calculated output power/correction, typically from -1.0 to 1.0.
     */
    public double calculate(double current) {
        // Handle first loop - reset timer and skip I/D terms to avoid stale timing
        if (firstLoop) {
            firstLoop = false;
            timer.reset();
            double error = target - current;
            lastError = error;
            lastTarget = target;
            return clamp(kP * error);
        }

        double elapsedTime = timer.seconds();
        timer.reset();

        // Calculate the error
        double error = target - current;

        double pTerm = kP * error;

        // Calculate the integral term with anti-windup clamping
        integralSum += error * elapsedTime;
        integralSum = clamp(integralSum, -maxIntegral, maxIntegral);

        // Calculate the derivative term (unfiltered)
        double derivative = (elapsedTime > 0.0001) ? (error - lastError) / elapsedTime : 0.0;
        lastError = error;

        // Feedforward based on rate of target change (velocity feedforward)
        double targetRate = (elapsedTime > 0.0001) ? (target - lastTarget) / elapsedTime : 0.0;
        lastTarget = target;
        double feedforward = kF * targetRate;

        // Combine all terms and clamp output to valid motor range
        double output = pTerm + (kI * integralSum) + (kD * derivative) + feedforward;

        return clamp(output);
    }

    /**
     * Sets the target value for the controller.
     *
     * @param target The desired setpoint for the system.
     */
    public void setTarget(double target) {
        this.target = target;
    }

    /**
     * Gets the current target of the controller.
     * @return The current target setpoint.
     */
    public double getTarget() {
        return this.target;
    }

    /**
     * Resets the integral sum and error history. Call this when disabling the controller
     * or when the target changes abruptly.
     */
    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastTarget = target;
        firstLoop = true;
        timer.reset();
    }

    private double clamp(double value) {
        return clamp(value, -1.0, 1.0);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    public double getP() { return this.kP; }
    public double getI() { return this.kI; }
    public double getD() { return this.kD; }
    public double getF() { return this.kF; }

    /**
     * Allows for live updating of the PIDF gains.
     * @param kP Proportional gain.
     * @param kI Integral gain.
     * @param kD Derivative gain.
     * @param kF Feedforward gain.
     */
    public void setGains(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
}
