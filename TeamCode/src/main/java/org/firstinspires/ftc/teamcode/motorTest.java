package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "Motor Test CCW", group = "Test")
public class motorTest extends LinearOpMode {

    static final DcMotorSimple.Direction CCW_DIRECTION = DcMotorSimple.Direction.FORWARD;
    static final double INTAKE_MAX_RPM = 5000;
    // The intake's encoder counts opposite to its drive direction (log: +power, negative velocity), which
    // makes the hub's built-in velocity PID run away. So the intake uses its own PI loop in software,
    // with the encoder sign flipped. If you fix the encoder wiring, set the sign to 1.
    static final double INTAKE_ENCODER_SIGN = -1;
    static final double INTAKE_FULL_POWER_RPM = 6000;   // measured: ~0.93 power gave ~5600 RPM
    static final double INTAKE_kP = 0.0002;             // power per RPM of error
    static final double INTAKE_kI = 0.0004;             // power per RPM*second of error

    // Encoder ticks per output-shaft revolution. 28 is the bare 1:1 Yellow Jacket (5000-6000 RPM).
    // Geared versions: 5.2:1 = 145.1, 13.7:1 = 383.6, 19.2:1 = 537.7, 26.9:1 = 751.8, 50.9:1 = 1425.1.
    static final double TICKS_PER_REV = 28;

    // Shooter velocity PIDF. F is feedforward from the motor's max speed; P/I/D are starting
    // values come from the Shooter PID Tune auto-tune run (best of 8 trials) - adjust while watching target vs actual RPM.
    static final double SHOOTER_TICKS_PER_REV = 28;
    static final double SHOOTER_MAX_RPM = 5300;
    static final double SHOOTER_kP = 50.0;
    static final double SHOOTER_kI = 0.5;
    static final double SHOOTER_kD = 0.0;
    static final double SHOOTER_kF = 32767.0 / (SHOOTER_MAX_RPM * SHOOTER_TICKS_PER_REV / 60.0);
    private static final double RPM_STEP = 100;

    // Flip to REVERSE if shooter2 spins the wrong way for how it's mounted.
    private static final DcMotorSimple.Direction SHOOTER2_DIRECTION = DcMotorSimple.Direction.FORWARD;
    // shooter1 and shooter2 are driven with the same power from one software copy of the tuned shooter PIDF
    // (SHOOTER_kP/kI/kD/kF, hub units: output 32767 = full power, error in ticks/sec), fed by shooter1's
    // encoder only. shooter2's encoder is ignored.
    private static final double HUB_FULL_SCALE = 32767.0;

    // Hood servo position change per A/B press (servo range is 0.0 - 1.0).
    private static final double HOOD_STEP = 0.05;

    @Override
    public void runOpMode() {

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setDirection(CCW_DIRECTION);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // "shooter1" must match the device name in your Robot Configuration.
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // encoder still readable; power set below
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // "shooter2" must match the device name in your Robot Configuration.
        // It gets the same power as shooter1 (shooter2's encoder is ignored).
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setDirection(SHOOTER2_DIRECTION);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        double shooter2Integral = 0;
        double lastErrTps = 0;
        ElapsedTime shooter2Timer = new ElapsedTime();

        // "hood" must match the servo name in your Robot Configuration. Starts at 0.
        Servo hood = hardwareMap.get(Servo.class, "hood");
        double hoodPosition = 0;
        hood.setPosition(hoodPosition);

        double targetRpm = 0;
        double intakeTargetRpm = INTAKE_MAX_RPM;
        double intakeIntegral = 0;
        ElapsedTime loopTimer = new ElapsedTime();
        boolean lastUp = false;
        boolean lastDown = false;
        boolean lastRight = false;
        boolean lastLeft = false;
        boolean lastA = false;
        boolean lastB = false;

        telemetry.addLine("Ready - press START. D-pad up/down = shooter RPM, right/left = intake RPM, A/B = hood down/up");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // D-pad right/left: change intake target RPM (one step per press)
            if (gamepad1.dpad_right && !lastRight) {
                intakeTargetRpm = Math.min(intakeTargetRpm + RPM_STEP, INTAKE_MAX_RPM);
            }
            if (gamepad1.dpad_left && !lastLeft) {
                intakeTargetRpm = Math.max(intakeTargetRpm - RPM_STEP, 0);
            }
            lastRight = gamepad1.dpad_right;
            lastLeft = gamepad1.dpad_left;

            // Software PI + feedforward for the intake (encoder sign corrected)
            double dt = loopTimer.seconds();
            loopTimer.reset();
            double intakeRpm = INTAKE_ENCODER_SIGN * motor.getVelocity() * 60.0 / TICKS_PER_REV;
            double intakeErr = intakeTargetRpm - intakeRpm;
            intakeIntegral += intakeErr * dt;
            double iLimit = 0.3 / INTAKE_kI;
            intakeIntegral = Math.max(-iLimit, Math.min(iLimit, intakeIntegral));
            double intakePower = intakeTargetRpm / INTAKE_FULL_POWER_RPM
                    + INTAKE_kP * intakeErr + INTAKE_kI * intakeIntegral;
            if (intakeTargetRpm <= 0) {
                intakePower = 0;
                intakeIntegral = 0;
            }
            motor.setPower(Math.max(0, Math.min(1, intakePower)));

            // D-pad up/down: change shooter target RPM (one step per press)
            if (gamepad1.dpad_up && !lastUp) {
                targetRpm = Math.min(targetRpm + RPM_STEP, SHOOTER_MAX_RPM);
            }
            if (gamepad1.dpad_down && !lastDown) {
                targetRpm = Math.max(targetRpm - RPM_STEP, 0);
            }
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            // A/B: lower/raise the hood servo (one step per press)
            if (gamepad1.a && !lastA) {
                hoodPosition = Math.max(hoodPosition - HOOD_STEP, 0);
            }
            if (gamepad1.b && !lastB) {
                hoodPosition = Math.min(hoodPosition + HOOD_STEP, 1);
            }
            lastA = gamepad1.a;
            lastB = gamepad1.b;
            hood.setPosition(hoodPosition);

            double shooterTps = shooter.getVelocity();
            double shooterRpm = shooterTps * 60.0 / SHOOTER_TICKS_PER_REV;

            // Tuned shooter PIDF run in software; the same power goes to both motors.
            double dt2 = shooter2Timer.seconds();
            shooter2Timer.reset();
            double pairPower = 0;
            if (targetRpm > 0) {
                double targetTps = targetRpm * SHOOTER_TICKS_PER_REV / 60.0;
                double errTps = targetTps - shooterTps;
                shooter2Integral += errTps * dt2;
                double iLimit2 = 0.3 * HUB_FULL_SCALE / SHOOTER_kI;   // cap the I term at 30% power
                shooter2Integral = Math.max(-iLimit2, Math.min(iLimit2, shooter2Integral));
                double derivative = dt2 > 0 ? (errTps - lastErrTps) / dt2 : 0;
                lastErrTps = errTps;
                pairPower = (SHOOTER_kF * targetTps + SHOOTER_kP * errTps
                        + SHOOTER_kI * shooter2Integral + SHOOTER_kD * derivative) / HUB_FULL_SCALE;
            } else {
                shooter2Integral = 0;   // power 0 with FLOAT: coast instead of braking
                lastErrTps = 0;
            }
            pairPower = Math.max(0, Math.min(1, pairPower));
            shooter.setPower(pairPower);
            shooter2.setPower(pairPower);

            telemetry.addData("Shooter target RPM", "%.0f", targetRpm);
            telemetry.addData("Shooter actual RPM", "%.1f", shooterRpm);
            telemetry.addData("Power (both motors)", "%.3f", pairPower);
            telemetry.addData("Hood position", "%.2f", hoodPosition);

            double ticksPerSec = INTAKE_ENCODER_SIGN * motor.getVelocity();
            double rpm = intakeRpm;
            double revolutions = INTAKE_ENCODER_SIGN * motor.getCurrentPosition() / TICKS_PER_REV;

            RobotLog.ii("IntakeDebug", "target=%.0f actual=%.1f power=%.3f pos=%d",
                    intakeTargetRpm, rpm, motor.getPower(), (int) (INTAKE_ENCODER_SIGN * motor.getCurrentPosition()));
            telemetry.addData("Intake target RPM", "%.0f", intakeTargetRpm);
            telemetry.addData("Ticks/sec", "%.1f", ticksPerSec);
            telemetry.addData("Intake actual RPM", "%.1f", rpm);
            telemetry.addData("Position (ticks)", (int) (INTAKE_ENCODER_SIGN * motor.getCurrentPosition()));
            telemetry.addData("Revolutions", "%.2f", revolutions);
            telemetry.update();
        }

        motor.setPower(0);
        shooter.setPower(0);
        shooter2.setPower(0);
    }
}
