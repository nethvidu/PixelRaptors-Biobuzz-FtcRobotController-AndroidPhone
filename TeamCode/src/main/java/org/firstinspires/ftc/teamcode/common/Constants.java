package org.firstinspires.ftc.teamcode.common;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing setup, copied from EuropePremierEventRobot's common/Constants.java.
 * Only the drive motor names are changed to match this Robot Configuration. The velocities,
 * Pinpoint pod offsets and follower gains are still that robot's tuning.
 */
public class Constants {

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .leftFrontMotorName("front_left")
            .leftRearMotorName("back_left")
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(80.7883642001415)
            .yVelocity(63.37828100009229);

    // Pinpoint Localization Configuration
    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .forwardPodY(-84.0)  // Perpendicular pod Y offset in mm (negative = behind center)
            .strafePodX(-153.0)  // Parallel pod X offset in mm (positive = right of center)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // FollowerConstants
    // These values can be tuned using the pedroPathing Tuning OpModes
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.2) // kg - robot mass
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.15, 0.0744913927044, 0.0015137578205253897))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.001, 0.02))
            .centripetalScaling(0);

    // PathConstraints
    // Constraints to be met for the path to be considered complete
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995, // how much of the path the robot must follow before it is considered complete. 0.0 <-> 1.0.
            100, // milliseconds, the follower has to correct after stopping
            1, // A higher value means stronger braking (0 to 1)
            0.1); // The higher the value, the earlier the braking starts (0 to 1)

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}
