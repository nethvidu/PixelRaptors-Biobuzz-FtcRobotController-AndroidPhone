package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.ShooterPIDF;

/**
 * Live PID tuning for shooter1 + shooter2 through FTC Dashboard (http://192.168.43.1:8080/dash while
 * connected to the Control Hub's Wi-Fi). Edit the fields below in the Dashboard's Configuration tab;
 * they apply immediately. Graph TargetVel / CurrentVel / OutputPower in the Dashboard's graph.
 *
 * Velocity is read from shooter1's encoder only and the same power goes to both motors.
 * D-pad up/down also change targetRpm by 100; D-pad right/left change intakeTargetRpm by 100.
 * The intake runs the same sign-corrected software PI as motorTest (its encoder counts backwards).
 */
@Config
@TeleOp(name = "Shooter PID Dashboard", group = "Test")
public class shooterDashboardTune extends OpMode {

    // Dashboard-editable. Velocity is in encoder ticks/sec (rpm * 28 / 60).
    public static double targetRpm = 0;
    public static double kP = 0.0015, kI = 0.000015, kD = 0.0;
    public static double kS = 0.0;
    public static double kV = 1.0 / (motorTest.SHOOTER_MAX_RPM * motorTest.SHOOTER_TICKS_PER_REV / 60.0);
    public static double RPM_STEP = 100;

    // Intake (Dashboard-editable). Software feedforward + PI, power per RPM.
    public static double intakeTargetRpm = motorTest.INTAKE_MAX_RPM;
    public static double intakeKP = motorTest.INTAKE_kP, intakeKI = motorTest.INTAKE_kI;
    public static double intakeFullPowerRpm = motorTest.INTAKE_FULL_POWER_RPM;

    // Written each loop so the Dashboard can show/graph them.
    public static double targetVelocity, velocity;

    private ShooterPIDF controller;
    private FtcDashboard dashboard;
    private DcMotorEx shooter;
    private DcMotorEx shooter2;
    private DcMotorEx intake;
    private boolean lastUp, lastDown, lastRight, lastLeft;
    private double intakeIntegral = 0;
    private final ElapsedTime intakeTimer = new ElapsedTime();

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // encoder still readable
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // FLOAT: let the flywheel coast down instead of braking.
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(motorTest.CCW_DIRECTION);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new ShooterPIDF(kP, kI, kD, kS, kV);
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !lastUp) targetRpm = Math.min(targetRpm + RPM_STEP, motorTest.SHOOTER_MAX_RPM);
        if (gamepad1.dpad_down && !lastDown) targetRpm = Math.max(targetRpm - RPM_STEP, 0);
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        if (gamepad1.dpad_right && !lastRight) intakeTargetRpm = Math.min(intakeTargetRpm + RPM_STEP, motorTest.INTAKE_MAX_RPM);
        if (gamepad1.dpad_left && !lastLeft) intakeTargetRpm = Math.max(intakeTargetRpm - RPM_STEP, 0);
        lastRight = gamepad1.dpad_right;
        lastLeft = gamepad1.dpad_left;

        // Intake: feedforward + PI on the sign-corrected encoder
        double intakeDt = intakeTimer.seconds();
        intakeTimer.reset();
        double intakeRpm = motorTest.INTAKE_ENCODER_SIGN * intake.getVelocity() * 60.0 / motorTest.TICKS_PER_REV;
        double intakeErr = intakeTargetRpm - intakeRpm;
        intakeIntegral += intakeErr * intakeDt;
        double intakeILimit = 0.3 / Math.max(intakeKI, 1e-9);
        intakeIntegral = Math.max(-intakeILimit, Math.min(intakeILimit, intakeIntegral));
        double intakePower = intakeTargetRpm / intakeFullPowerRpm + intakeKP * intakeErr + intakeKI * intakeIntegral;
        if (intakeTargetRpm <= 0) {
            intakePower = 0;
            intakeIntegral = 0;
        }
        intakePower = Math.max(0, Math.min(1, intakePower));
        intake.setPower(intakePower);

        velocity = shooter.getVelocity();   // shooter1's encoder only
        double rpm = velocity * 60.0 / motorTest.SHOOTER_TICKS_PER_REV;
        targetVelocity = targetRpm * motorTest.SHOOTER_TICKS_PER_REV / 60.0;

        double power = 0;
        if (targetRpm > 0) {
            controller.setGains(kP, kI, kD, kS, kV);
            controller.setTarget(targetVelocity);
            power = controller.calculate(velocity);
        } else {
            controller.reset();   // power 0 with FLOAT: coast
        }
        shooter.setPower(power);
        shooter2.setPower(power);

        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Actual RPM", "%.1f", rpm);
        telemetry.addData("Power", "%.3f", power);
        telemetry.addData("Intake RPM (target / actual)", "%.0f / %.1f", intakeTargetRpm, intakeRpm);

        // All values numeric so FTC Dashboard can graph them over time.
        Telemetry dash = dashboard.getTelemetry();
        dash.addData("TargetVel", targetVelocity);
        dash.addData("CurrentVel", velocity);
        dash.addData("TargetRPM", targetRpm);
        dash.addData("RPM", rpm);
        dash.addData("OutputPower", power);
        dash.addData("IntakeTargetRPM", intakeTargetRpm);
        dash.addData("IntakeRPM", intakeRpm);
        dash.addData("IntakePower", intakePower);
        dash.update();
    }

    @Override
    public void stop() {
        shooter.setPower(0);
        shooter2.setPower(0);
        intake.setPower(0);
    }
}
