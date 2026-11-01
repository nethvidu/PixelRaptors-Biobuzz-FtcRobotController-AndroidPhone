package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Runs only the intake motor, with the same software PI velocity control as motorTest.
 * D-pad up/down changes the target RPM, X stops it. Shows RPM (bare motor, 28 ticks/rev) and current draw.
 */
@TeleOp(name = "Intake Only", group = "Test")
public class intakeTest extends LinearOpMode {

    private static final double RPM_STEP = 100;

    @Override
    public void runOpMode() {

        // "intake" must match the device name in your Robot Configuration.
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(motorTest.CCW_DIRECTION);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double targetRpm = 0;
        double integral = 0;
        ElapsedTime loopTimer = new ElapsedTime();
        boolean lastUp = false;
        boolean lastDown = false;

        telemetry.addLine("Ready - press START. D-pad up/down = intake RPM, X = stop");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            // D-pad up/down: change intake target RPM (one step per press), X: stop
            if (gamepad1.dpad_up && !lastUp) {
                targetRpm = Math.min(targetRpm + RPM_STEP, motorTest.INTAKE_MAX_RPM);
            }
            if (gamepad1.dpad_down && !lastDown) {
                targetRpm = Math.max(targetRpm - RPM_STEP, 0);
            }
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            if (gamepad1.x) {
                targetRpm = 0;
            }

            // Software PI + feedforward (encoder sign corrected, see motorTest)
            double dt = loopTimer.seconds();
            loopTimer.reset();
            double rpm = motorTest.INTAKE_ENCODER_SIGN * intake.getVelocity() * 60.0 / motorTest.TICKS_PER_REV;
            double err = targetRpm - rpm;
            integral += err * dt;
            double iLimit = 0.3 / motorTest.INTAKE_kI;
            integral = Math.max(-iLimit, Math.min(iLimit, integral));
            double power = targetRpm / motorTest.INTAKE_FULL_POWER_RPM
                    + motorTest.INTAKE_kP * err + motorTest.INTAKE_kI * integral;
            if (targetRpm <= 0) {
                power = 0;
                integral = 0;
            }
            power = Math.max(0, Math.min(1, power));
            intake.setPower(power);

            telemetry.addData("Intake target RPM", "%.0f", targetRpm);
            telemetry.addData("Intake actual RPM", "%.1f", rpm);
            telemetry.addData("Power", "%.3f", power);
            telemetry.addData("Current (A)", "%.2f", intake.getCurrent(CurrentUnit.AMPS));
            // Raw encoder (no sign correction): if RPM reads negative while spinning, flip INTAKE_ENCODER_SIGN
            telemetry.addData("Raw velocity (ticks/s)", "%.1f", intake.getVelocity());
            telemetry.addData("Raw position (ticks)", intake.getCurrentPosition());
            telemetry.update();
        }

        intake.setPower(0);
    }
}
