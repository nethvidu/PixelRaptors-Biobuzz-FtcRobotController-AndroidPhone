package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Velocity PIDF for the flywheel. Standard PID on velocity error plus a feedforward of
 * {@code kS + kV * target} (static term + velocity term). Output is clamped to [0, 1] since the
 * flywheel only drives forward.
 *
 * <p>Time-aware (uses real dt), clamps the integral for anti-windup, and skips I/D on the first
 * loop to avoid a stale-dt spike.
 */
public class ShooterPIDF {

    private double kP, kI, kD, kS, kV;
    private double maxIntegral = 1.0;

    private double target;
    private double integralSum = 0;
    private double lastError = 0;
    private boolean firstLoop = true;
    private final ElapsedTime timer = new ElapsedTime();

    public ShooterPIDF(double kP, double kI, double kD, double kS, double kV) {
        setGains(kP, kI, kD, kS, kV);
    }

    /** @param targetVelocity the setpoint velocity (drives the kV feedforward). */
    public void setTarget(double targetVelocity) {
        this.target = targetVelocity;
    }

    public double getTarget() {
        return target;
    }

    /**
     * @param measurement the current measured velocity
     * @return output clamped to [0, 1]
     */
    public double calculate(double measurement) {
        double error = target - measurement;
        double feedforward = kS + kV * target;

        if (firstLoop) {
            firstLoop = false;
            timer.reset();
            lastError = error;
            return clamp(kP * error + feedforward);
        }

        double dt = timer.seconds();
        timer.reset();

        integralSum = Range.clip(integralSum + error * dt, -maxIntegral, maxIntegral);
        double derivative = (dt > 1e-4) ? (error - lastError) / dt : 0.0;
        lastError = error;

        return clamp(kP * error + kI * integralSum + kD * derivative + feedforward);
    }

    public void setGains(double kP, double kI, double kD, double kS, double kV) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
    }

    /** Clears integral, error history, and timing so the next call behaves like a first loop. */
    public void reset() {
        integralSum = 0;
        lastError = 0;
        firstLoop = true;
        timer.reset();
    }

    private double clamp(double value) {
        return Range.clip(value, 0.0, 1.0);
    }
}
