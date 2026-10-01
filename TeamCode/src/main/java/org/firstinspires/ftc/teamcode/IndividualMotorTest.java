package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * This OpMode lets you run one drive motor at a time, using the face buttons on gamepad1.
 * This is useful for confirming that each motor is wired to the correct port and spins the
 * correct direction before you drive the robot as a whole.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_drive, front_right_drive, back_left_drive, back_right_drive
 *
 * Button mapping (Logitech gamepad):
 *   Y = front left
 *   A = back left
 *   X = front right
 *   B = back right
 *
 * Hold a button to run that motor, release to stop it. Only one motor runs at a time.
 */
@TeleOp(name = "IndividualMotorTest", group = "Robot")
public class IndividualMotorTest extends OpMode {
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    static final double MOTOR_POWER = 0.5;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        frontLeftDrive.setPower(gamepad1.y ? MOTOR_POWER : 0);
        backLeftDrive.setPower(gamepad1.a ? MOTOR_POWER : 0);
        frontRightDrive.setPower(gamepad1.x ? MOTOR_POWER : 0);
        backRightDrive.setPower(gamepad1.b ? MOTOR_POWER : 0);

        telemetry.addData("Front Left (Y)", gamepad1.y ? "RUNNING" : "stopped");
        telemetry.addData("Back Left (A)", gamepad1.a ? "RUNNING" : "stopped");
        telemetry.addData("Front Right (X)", gamepad1.x ? "RUNNING" : "stopped");
        telemetry.addData("Back Right (B)", gamepad1.b ? "RUNNING" : "stopped");
        telemetry.update();
    }
}
