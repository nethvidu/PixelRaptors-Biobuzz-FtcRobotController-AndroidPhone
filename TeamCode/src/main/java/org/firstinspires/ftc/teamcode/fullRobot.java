package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Whole robot: Pedro Pathing mecanum drive (Pinpoint localizer), turret, shooter (shooter1 + shooter2),
 * hood, door and intake. The robot starts at (72, 72) on the field.
 * Shooter/intake control and tuning constants are the same as motorTest / shooterHoodIntake.
 *
 * Gamepad 1 (driver):
 *   Left stick       = drive (field-centric: forward = the way the robot faced at start)
 *   Right stick X    = turn
 *   Right trigger    = slow mode (hold)
 *   A                = reset field-centric forward to the robot's current heading
 *   LB / RB          = turret 5 deg left / right
 *   Y                = turret forward
 *
 * Gamepad 2 (operator), same layout as Shooter Hood Intake:
 *   D-pad up/down    = shooter RPM
 *   Right trigger    = run intake at its set RPM while held, stop on release
 *   D-pad right/left = intake set RPM
 *   LB / RB          = hood down / up
 *   Y                = toggle door open / closed
 *   X / B            = door close position down / up (fine adjust)
 *   A                = stop shooter
 *
 * The turret calibrates forward from its absolute encoder at init, so start it pointing roughly forward.
 */
@TeleOp(name = "Full Robot", group = "Test")
public class fullRobot extends LinearOpMode {

    private static final Pose START_POSE = new Pose(72, 72, Math.toRadians(0));
    private static final double SLOW_MODE_SCALE = 0.35;
    private static final double TURRET_STEP_DEG = 5;
    private static final double TURRET_FORWARD_DEG = 90;
    private static final double SHOOTER_RPM_STEP = 100;
    private static final double INTAKE_RPM_STEP = 100;
    private static final double HOOD_STEP = 0.05;
    private static final double DOOR_OPEN = 0.19;
    private static final double DOOR_CLOSED = 0.46;
    private static final double DOOR_CLOSED_STEP = 0.01;
    private static final double HUB_FULL_SCALE = 32767.0;

    @Override
    public void runOpMode() {

        // Device names must match the Robot Configuration.

        // --- Drive: Pedro follower (front_left/back_left/front_right/back_right + pinpoint, see Constants) ---
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        double fieldOrientationOffset = START_POSE.getHeading();

        // --- Turret ("turret" motor + "octoquad") ---
        Turret turret = new Turret(hardwareMap);
        double turretTargetDeg = TURRET_FORWARD_DEG;

        // --- Intake ---
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(motorTest.CCW_DIRECTION);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // --- Shooter: shooter1's encoder drives the PIDF; shooter2 gets the same power ---
        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // --- Hood and door ---
        Servo hood = hardwareMap.get(Servo.class, "hood");
        double hoodPosition = 0;
        hood.setPosition(hoodPosition);

        Servo door = hardwareMap.get(Servo.class, "door");
        boolean doorOpen = false;
        double doorClosed = DOOR_CLOSED;
        door.setPosition(doorClosed);

        double shooterTargetRpm = 0;
        double shooterIntegral = 0;
        double lastErrTps = 0;
        double intakeSetRpm = motorTest.INTAKE_MAX_RPM;   // used while the right trigger is held
        double intakeIntegral = 0;
        ElapsedTime loopTimer = new ElapsedTime();

        // Gamepad 1 edge detection
        boolean lastLB1 = false, lastRB1 = false, lastA1 = false;
        // Gamepad 2 edge detection
        boolean lastUp = false, lastDown = false, lastRight = false, lastLeft = false;
        boolean lastLB = false, lastRB = false, lastX = false, lastB = false, lastY = false;

        telemetry.addLine("Ready - gamepad 1 = drive + turret, gamepad 2 = shooter/intake/hood/door");
        telemetry.addData("Turret (deg)", "%.1f", Math.toDegrees(turret.getCurrentAngle()));
        telemetry.update();

        waitForStart();
        follower.startTeleopDrive(true);   // brake mode
        loopTimer.reset();

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.getPose();
            double heading = pose.getHeading();

            // ================= Gamepad 1: drive + turret =================
            if (gamepad1.a && !lastA1) {
                fieldOrientationOffset = heading;
            }
            lastA1 = gamepad1.a;

            double scale = gamepad1.right_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;
            double forward = -gamepad1.left_stick_y * scale;
            double strafe = -gamepad1.left_stick_x * scale;
            double rx = -gamepad1.right_stick_x * scale;

            // Field-centric (same as EuropePremierEventRobot's MainTeleOp): rotate the stick input
            // into the robot frame, then drive robot-centric.
            double rotationAngle = fieldOrientationOffset - heading;
            follower.setTeleOpDrive(
                    forward * Math.cos(rotationAngle) - strafe * Math.sin(rotationAngle),
                    forward * Math.sin(rotationAngle) + strafe * Math.cos(rotationAngle),
                    rx,
                    true);

            if (gamepad1.left_bumper && !lastLB1) turretTargetDeg += TURRET_STEP_DEG;
            if (gamepad1.right_bumper && !lastRB1) turretTargetDeg -= TURRET_STEP_DEG;
            lastLB1 = gamepad1.left_bumper;
            lastRB1 = gamepad1.right_bumper;
            if (gamepad1.y) turretTargetDeg = TURRET_FORWARD_DEG;

            turret.setTargetAngle(Math.toRadians(turretTargetDeg));
            turret.update(heading);

            // ================= Gamepad 2: shooter, intake, hood, door =================
            if (gamepad2.dpad_up && !lastUp) {
                shooterTargetRpm = Math.min(shooterTargetRpm + SHOOTER_RPM_STEP, motorTest.SHOOTER_MAX_RPM);
            }
            if (gamepad2.dpad_down && !lastDown) {
                shooterTargetRpm = Math.max(shooterTargetRpm - SHOOTER_RPM_STEP, 0);
            }
            if (gamepad2.dpad_right && !lastRight) {
                intakeSetRpm = Math.min(intakeSetRpm + INTAKE_RPM_STEP, motorTest.INTAKE_MAX_RPM);
            }
            if (gamepad2.dpad_left && !lastLeft) {
                intakeSetRpm = Math.max(intakeSetRpm - INTAKE_RPM_STEP, 0);
            }
            if (gamepad2.left_bumper && !lastLB) {
                hoodPosition = Math.max(hoodPosition - HOOD_STEP, 0);
            }
            if (gamepad2.right_bumper && !lastRB) {
                hoodPosition = Math.min(hoodPosition + HOOD_STEP, 1);
            }
            if (gamepad2.x && !lastX) {
                doorClosed = Math.max(doorClosed - DOOR_CLOSED_STEP, 0);
            }
            if (gamepad2.b && !lastB) {
                doorClosed = Math.min(doorClosed + DOOR_CLOSED_STEP, 1);
            }
            if (gamepad2.y && !lastY) {
                doorOpen = !doorOpen;
            }
            if (gamepad2.a) {
                shooterTargetRpm = 0;
            }
            lastUp = gamepad2.dpad_up;
            lastDown = gamepad2.dpad_down;
            lastRight = gamepad2.dpad_right;
            lastLeft = gamepad2.dpad_left;
            lastLB = gamepad2.left_bumper;
            lastRB = gamepad2.right_bumper;
            lastX = gamepad2.x;
            lastB = gamepad2.b;
            lastY = gamepad2.y;

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

            // --- Intake: runs only while the right trigger is held; software PI + feedforward ---
            double intakeTargetRpm = gamepad2.right_trigger > 0.5 ? intakeSetRpm : 0;
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

            // --- Hood and door ---
            hood.setPosition(hoodPosition);
            door.setPosition(doorOpen ? DOOR_OPEN : doorClosed);

            telemetry.addData("Pose", "x=%.1f y=%.1f h=%.0f deg", pose.getX(), pose.getY(), Math.toDegrees(heading));
            telemetry.addData("Turret target / actual (deg)", "%.1f / %.1f",
                    Math.toDegrees(turret.getTargetAngle()), Math.toDegrees(turret.getCurrentAngle()));
            telemetry.addData("Shooter target / actual RPM", "%.0f / %.1f", shooterTargetRpm, shooterRpm);
            telemetry.addData("Shooter power", "%.3f", shooterPower);
            telemetry.addData("Intake set / target / actual RPM", "%.0f / %.0f / %.1f", intakeSetRpm, intakeTargetRpm, intakeRpm);
            telemetry.addData("Intake current (A)", "%.2f", intake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Hood position", "%.2f", hoodPosition);
            telemetry.addData("Door", "%s (close pos %.2f)", doorOpen ? "OPEN" : "CLOSED", doorClosed);
            telemetry.update();
        }

        follower.setTeleOpDrive(0, 0, 0, true);
        follower.update();
        turret.moveMotor(0);
        shooter1.setPower(0);
        shooter2.setPower(0);
        intake.setPower(0);
    }
}
