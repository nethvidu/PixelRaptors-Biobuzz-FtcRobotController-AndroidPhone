package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Operates the shooter (shooter1 + shooter2), hood servo and intake, using the same control and
 * tuning constants as motorTest. Everything starts stopped, hood at 0.
 *   D-pad up/down    = shooter RPM
 *   D-pad right/left = intake RPM
 *   A / B            = hood down / up
 *   X                = stop shooter and intake
 */
@TeleOp(name = "Shooter Hood Intake", group = "Test")
public class shooterHoodIntake extends LinearOpMode {

    private static final double SHOOTER_RPM_STEP = 100;
    private static final double INTAKE_RPM_STEP = 100;
    private static final double HOOD_STEP = 0.05;
    private static final double HUB_FULL_SCALE = 32767.0;

    @Override
    public void runOpMode() {

        // Device names must match the Robot Configuration.
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(motorTest.CCW_DIRECTION);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // shooter1's encoder drives the PIDF; shooter2 gets the same power (its encoder is ignored).
        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Servo hood = hardwareMap.get(Servo.class, "hood");
        double hoodPosition = 0;
        hood.setPosition(hoodPosition);

        double shooterTargetRpm = 0;
        double shooterIntegral = 0;
        double lastErrTps = 0;
        double intakeTargetRpm = 0;
        double intakeIntegral = 0;
        ElapsedTime loopTimer = new ElapsedTime();

        boolean lastUp = false, lastDown = false, lastRight = false, lastLeft = false;
        boolean lastA = false, lastB = false;

        telemetry.addLine("Ready - D-pad up/down = shooter, right/left = intake, A/B = hood, X = stop");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            // --- Controls (one step per press) ---
            if (gamepad1.dpad_up && !lastUp) {
                shooterTargetRpm = Math.min(shooterTargetRpm + SHOOTER_RPM_STEP, motorTest.SHOOTER_MAX_RPM);
            }
            if (gamepad1.dpad_down && !lastDown) {
                shooterTargetRpm = Math.max(shooterTargetRpm - SHOOTER_RPM_STEP, 0);
            }
            if (gamepad1.dpad_right && !lastRight) {
                intakeTargetRpm = Math.min(intakeTargetRpm + INTAKE_RPM_STEP, motorTest.INTAKE_MAX_RPM);
            }
            if (gamepad1.dpad_left && !lastLeft) {
                intakeTargetRpm = Math.max(intakeTargetRpm - INTAKE_RPM_STEP, 0);
            }
            if (gamepad1.a && !lastA) {
                hoodPosition = Math.max(hoodPosition - HOOD_STEP, 0);
            }
            if (gamepad1.b && !lastB) {
                hoodPosition = Math.min(hoodPosition + HOOD_STEP, 1);
            }
            if (gamepad1.x) {
                shooterTargetRpm = 0;
                intakeTargetRpm = 0;
            }
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastRight = gamepad1.dpad_right;
            lastLeft = gamepad1.dpad_left;
            lastA = gamepad1.a;
            lastB = gamepad1.b;

            double dt = loopTimer.seconds();
            loopTimer.reset();

            // --- Shooter: tuned PIDF in software (hub units), same power to both motors ---
            double shooterTps = motorTest.SHOOTER_ENCODER_SIGN * shooter1.getVelocity();
            double shooterRpm = shooterTps * 60.0 / motorTest.SHOOTER_TICKS_PER_REV;
            double shooterPower = 0;
            if (shooterTargetRpm > 0) {
                double targetTps = shooterTargetRpm * motorTest.SHOOTER_TICKS_PER_REV / 60.0;
                double errTps = targetTps - shooterTps;
                shooterIntegral += errTps * dt;
                double iLimit = 0.3 * HUB_FULL_SCALE / motorTest.SHOOTER_kI;   // cap the I term at 30% power
                shooterIntegral = Math.max(-iLimit, Math.min(iLimit, shooterIntegral));
                double derivative = dt > 0 ? (errTps - lastErrTps) / dt : 0;
                lastErrTps = errTps;
                shooterPower = (motorTest.SHOOTER_kF * targetTps + motorTest.SHOOTER_kP * errTps
                        + motorTest.SHOOTER_kI * shooterIntegral + motorTest.SHOOTER_kD * derivative) / HUB_FULL_SCALE;
            } else {
                shooterIntegral = 0;   // power 0 with FLOAT: coast instead of braking
                lastErrTps = 0;
            }
            shooterPower = Math.max(0, Math.min(1, shooterPower));
            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);

            // --- Intake: software PI + feedforward ---
            double intakeRpm = motorTest.INTAKE_ENCODER_SIGN * intake.getVelocity() * 60.0 / motorTest.TICKS_PER_REV;
            double intakePower = 0;
            if (intakeTargetRpm > 0) {
                double intakeErr = intakeTargetRpm - intakeRpm;
                intakeIntegral += intakeErr * dt;
                double iLimit = 0.3 / motorTest.INTAKE_kI;
                intakeIntegral = Math.max(-iLimit, Math.min(iLimit, intakeIntegral));
                intakePower = intakeTargetRpm / motorTest.INTAKE_FULL_POWER_RPM
                        + motorTest.INTAKE_kP * intakeErr + motorTest.INTAKE_kI * intakeIntegral;
            } else {
                intakeIntegral = 0;
            }
            intakePower = Math.max(0, Math.min(1, intakePower));
            intake.setPower(intakePower);

            // --- Hood ---
            hood.setPosition(hoodPosition);

            telemetry.addData("Shooter target RPM", "%.0f", shooterTargetRpm);
            telemetry.addData("Shooter actual RPM", "%.1f", shooterRpm);
            telemetry.addData("Shooter power", "%.3f", shooterPower);
            telemetry.addData("Intake target RPM", "%.0f", intakeTargetRpm);
            telemetry.addData("Intake actual RPM", "%.1f", intakeRpm);
            telemetry.addData("Intake current (A)", "%.2f", intake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Hood position", "%.2f", hoodPosition);
            telemetry.update();
        }

        shooter1.setPower(0);
        shooter2.setPower(0);
        intake.setPower(0);
    }
}
