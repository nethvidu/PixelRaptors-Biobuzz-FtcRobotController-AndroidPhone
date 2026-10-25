package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Auto-tunes shooter1's velocity PID. For each (kP, kI) candidate it runs a step-response test,
 * scores it (lower is better), then reports the best pair. Read results with:
 *   adb logcat -s ShooterTune
 * Spins up to 4000 RPM - keep the area clear.
 */
@TeleOp(name = "Shooter PID Tune", group = "Test")
public class shooterTune extends LinearOpMode {

    private static final String TAG = "ShooterTune";
    private static final double[] STEP_RPMS = {500, 1000, 2000, 4000};
    private static final double STEP_SECONDS = 2.0;
    private static final double SPINDOWN_SECONDS = 6.0;

    private static final double[] KP_CANDIDATES = {5, 12, 25, 50};
    private static final double[] KI_CANDIDATES = {0.5, 3};
    private static final double KD = 0.0;

    @Override
    public void runOpMode() {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Ready - press START to auto-tune (spins up to 4000 RPM)");
        telemetry.update();
        waitForStart();

        double bestScore = Double.MAX_VALUE;
        double bestP = 0, bestI = 0;
        ElapsedTime clock = new ElapsedTime();

        for (double kP : KP_CANDIDATES) {
            for (double kI : KI_CANDIDATES) {
                if (!opModeIsActive()) break;
                shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(kP, kI, KD, motorTest.SHOOTER_kF));
                RobotLog.ii(TAG, "TRIAL_BEGIN kP=%.2f kI=%.2f kD=%.2f kF=%.3f", kP, kI, KD, motorTest.SHOOTER_kF);

                double errSum = 0, overshootSum = 0;
                int samples = 0, upSteps = 0;

                for (double targetRpm : STEP_RPMS) {
                    if (!opModeIsActive()) break;
                    shooter.setVelocity(targetRpm * motorTest.SHOOTER_TICKS_PER_REV / 60.0);
                    double peak = 0;
                    ElapsedTime step = new ElapsedTime();
                    while (opModeIsActive() && step.seconds() < STEP_SECONDS) {
                        double actual = shooter.getVelocity() * 60.0 / motorTest.SHOOTER_TICKS_PER_REV;
                        RobotLog.ii(TAG, "DATA kP=%.2f kI=%.2f t=%.3f target=%.1f actual=%.1f power=%.3f",
                                kP, kI, clock.seconds(), targetRpm, actual, shooter.getPower());
                        errSum += Math.abs(targetRpm - actual);
                        peak = Math.max(peak, actual);
                        samples++;
                        sleep(20);
                    }
                    if (targetRpm > 0) {
                        overshootSum += Math.max(0, peak - targetRpm);
                        upSteps++;
                    }
                }

                // Coast down (zero-power FLOAT) instead of driving the PID to 0, so it never brakes.
                shooter.setPower(0);
                sleep((long) (SPINDOWN_SECONDS * 1000));

                double meanErr = samples > 0 ? errSum / samples : Double.MAX_VALUE;
                double meanOvershoot = upSteps > 0 ? overshootSum / upSteps : 0;
                double score = meanErr + 0.5 * meanOvershoot;
                RobotLog.ii(TAG, "TRIAL_END kP=%.2f kI=%.2f meanErr=%.1f meanOvershoot=%.1f score=%.1f",
                        kP, kI, meanErr, meanOvershoot, score);
                if (score < bestScore) {
                    bestScore = score;
                    bestP = kP;
                    bestI = kI;
                }
                telemetry.addData("Last trial", "kP=%.1f kI=%.1f score=%.1f", kP, kI, score);
                telemetry.addData("Best so far", "kP=%.1f kI=%.1f score=%.1f", bestP, bestI, bestScore);
                telemetry.update();
            }
        }

        shooter.setPower(0);
        RobotLog.ii(TAG, "BEST kP=%.2f kI=%.2f kD=%.2f score=%.1f", bestP, bestI, KD, bestScore);
        telemetry.addData("DONE - best", "kP=%.1f kI=%.1f", bestP, bestI);
        telemetry.update();
        sleep(3000);
    }
}
