package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Turret test. Left/right bumper turns the target left/right by TARGET_STEP_DEG per press, Y returns to forward.
 * TARGET_DEG and the Turret PIDF gains (kP/kI/kD/kF) can also be changed live in FTC Dashboard
 * (http://192.168.43.1:8080/dash). Angles are canonical: 0 = robot's right, CCW positive,
 * 90 = forward, 180 = left.
 *
 * The turret calibrates its forward-zero from the absolute encoder at init, so start it pointing
 * roughly forward.
 */
@Config
@TeleOp(name = "Turret Test", group = "Test")
public class turretTest extends OpMode {

    public static double TARGET_DEG = 90;
    public static double TARGET_STEP_DEG = 5;

    private Turret turret;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turret = new Turret(hardwareMap);
        TARGET_DEG = 90;
        telemetry.addLine("Ready - left/right bumper = turn turret 5 deg left/right, Y = forward");
        telemetry.addData("Current (deg)", "%.1f", Math.toDegrees(turret.getCurrentAngle()));
        telemetry.update();
    }

    @Override
    public void loop() {
        // Left/right bumper: turn left (CCW) / right (CW) one step per press, Y: back to forward
        if (gamepad1.left_bumper && !lastLeftBumper) TARGET_DEG += TARGET_STEP_DEG;
        if (gamepad1.right_bumper && !lastRightBumper) TARGET_DEG -= TARGET_STEP_DEG;
        lastLeftBumper = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;
        if (gamepad1.y) TARGET_DEG = 90;

        turret.setTargetAngle(Math.toRadians(TARGET_DEG));
        turret.update(0);   // no localizer: heading feedforward unused

        telemetry.addData("Target (deg)", "%.1f", Math.toDegrees(turret.getTargetAngle()));
        telemetry.addData("Current (deg)", "%.1f", Math.toDegrees(turret.getCurrentAngle()));
        telemetry.addData("Raw angle (deg)", "%.1f", Math.toDegrees(turret.getRawAngle()));
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.moveMotor(0);
    }
}
