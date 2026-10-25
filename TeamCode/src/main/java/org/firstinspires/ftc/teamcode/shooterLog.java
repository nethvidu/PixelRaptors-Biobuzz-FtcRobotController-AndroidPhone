package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Runs shooter1 + motor2 with the tuned shooter PID (same control as motorTest) and logs the PID
 * response to logcat while you feed balls through. Read with:  adb logcat -s ShooterLog
 *
 *   D-pad up/down : shooter target RPM (+/- 100)
 *   A             : start / stop logging (toggle)
 *
 * The PID adapts to load, not to detecting balls. Load shows up as the flywheel running below target,
 * so the (smoothed) RPM shortfall is the load, and kP/kI are scaled up with it so a loaded flywheel
 * gets a stronger correction. No current sensing. While logging, every loop writes target / actual /
 * error / power / load / gain scale.
 */
@TeleOp(name = "Shooter PID Log", group = "Test")
public class shooterLog extends LinearOpMode {

    private static final String TAG = "ShooterLog";
    private static final double RPM_STEP = 100;
    private static final double HUB_FULL_SCALE = 32767.0;
    // Load = smoothed RPM shortfall below target. kP/kI are multiplied by 1 + load / LOAD_RPM_FOR_2X.
    private static final double LOAD_RPM_FOR_2X = 200;   // 200 RPM below target doubles the gains
    private static final double MAX_GAIN_SCALE = 3.0;    // never more than 3x the tuned gains
    private static final double LOAD_ALPHA = 0.2;        // smoothing of the shortfall

    @Override
    public void runOpMode() {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double targetRpm = 0;
        double integral = 0;
        double lastErrTps = 0;
        boolean lastUp = false, lastDown = false, lastA = false;

        boolean logging = false;
        double loadRpm = 0;   // smoothed RPM shortfall below target
        ElapsedTime logClock = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();

        telemetry.addLine("Ready - D-pad up/down = RPM, A = start/stop logging");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !lastUp) targetRpm = Math.min(targetRpm + RPM_STEP, motorTest.SHOOTER_MAX_RPM);
            if (gamepad1.dpad_down && !lastDown) targetRpm = Math.max(targetRpm - RPM_STEP, 0);
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            if (gamepad1.a && !lastA) {
                logging = !logging;
                if (logging) {
                    logClock.reset();
                    RobotLog.ii(TAG, "LOG_START target=%.0f kP=%.2f kI=%.2f kD=%.2f kF=%.3f",
                            targetRpm, motorTest.SHOOTER_kP, motorTest.SHOOTER_kI,
                            motorTest.SHOOTER_kD, motorTest.SHOOTER_kF);
                } else {
                    RobotLog.ii(TAG, "LOG_STOP");
                }
            }
            lastA = gamepad1.a;

            // Same tuned PIDF as motorTest, run in software on shooter1's encoder, same power to both motors.
            double dt = loopTimer.seconds();
            loopTimer.reset();
            double tps = shooter.getVelocity();
            double actualRpm = tps * 60.0 / motorTest.SHOOTER_TICKS_PER_REV;
            double power = 0;
            double errRpm = targetRpm - actualRpm;

            // Load: how far below target the flywheel is being held (never negative)
            double shortfall = targetRpm > 0 ? Math.max(0, errRpm) : 0;
            loadRpm += LOAD_ALPHA * (shortfall - loadRpm);
            double gainScale = Math.min(MAX_GAIN_SCALE, 1 + loadRpm / LOAD_RPM_FOR_2X);

            if (targetRpm > 0) {
                double targetTps = targetRpm * motorTest.SHOOTER_TICKS_PER_REV / 60.0;
                double errTps = targetTps - tps;
                integral += errTps * dt;
                double kP = motorTest.SHOOTER_kP * gainScale;
                double kI = motorTest.SHOOTER_kI * gainScale;
                double iLimit = 0.3 * HUB_FULL_SCALE / kI;
                integral = Math.max(-iLimit, Math.min(iLimit, integral));
                double derivative = dt > 0 ? (errTps - lastErrTps) / dt : 0;
                lastErrTps = errTps;
                power = (motorTest.SHOOTER_kF * targetTps + kP * errTps
                        + kI * integral + motorTest.SHOOTER_kD * derivative) / HUB_FULL_SCALE;
            } else {
                integral = 0;
                lastErrTps = 0;
            }
            power = Math.max(0, Math.min(1, power));
            shooter.setPower(power);
            motor2.setPower(power);

            if (logging) {
                RobotLog.ii(TAG, "DATA t=%.3f target=%.0f actual=%.1f error=%.1f power=%.3f loadRpm=%.1f gainScale=%.2f",
                        logClock.seconds(), targetRpm, actualRpm, errRpm, power, loadRpm, gainScale);
            }

            telemetry.addData("Logging", logging ? "ON (A to stop)" : "OFF (A to start)");
            telemetry.addData("Target RPM", "%.0f", targetRpm);
            telemetry.addData("Actual RPM", "%.1f", actualRpm);
            telemetry.addData("Power", "%.3f", power);
            telemetry.addData("Load (RPM below target)", "%.0f", loadRpm);
            telemetry.addData("PID gain scale", "%.2f", gainScale);
            telemetry.update();
        }

        if (logging) RobotLog.ii(TAG, "LOG_STOP");
        shooter.setPower(0);
        motor2.setPower(0);
    }
}
