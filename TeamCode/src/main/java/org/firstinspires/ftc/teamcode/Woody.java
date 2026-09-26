package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.SortOrder;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

/*
 * This OpMode is BuzzLightyear's mecanum drive combined with BallColorLocator's ball
 * detection: drive the robot with gamepad1 like normal, while the phone's camera keeps
 * looking for the red, blue, and yellow BioBuzz balls and reports what it sees in telemetry.
 *
 * This OpMode assumes the same four drive motors as BuzzLightyear:
 *   front_left_drive, front_right_drive, back_left_drive, back_right_drive
 */
@TeleOp(name = "Woody", group = "Robot")
public class Woody extends OpMode {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // Minimum/maximum contour area (in pixels) a blob must have to be considered a ball,
    // at 320x240 resolution. Tune these once you see real ball sizes in the telemetry.
    static final double MIN_BALL_AREA = 150;
    static final double MAX_BALL_AREA = 20000;
    static final double MIN_CIRCULARITY = 0.7;

    ColorBlobLocatorProcessor redLocator;
    ColorBlobLocatorProcessor blueLocator;
    ColorBlobLocatorProcessor yellowLocator;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // We set the right motors in reverse which is needed for drive trains where the right
        // motors are opposite to the left ones. (Confirmed with IndividualMotorTest.)
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        redLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.RED)
                .setBlurSize(5)
                .build();
        applyBallFilters(redLocator);

        blueLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.BLUE)
                .setBlurSize(5)
                .build();
        applyBallFilters(blueLocator);

        yellowLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.YELLOW)
                .setBlurSize(5)
                .build();
        applyBallFilters(yellowLocator);

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(redLocator)
                .addProcessor(blueLocator)
                .addProcessor(yellowLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(BuiltinCameraDirection.BACK)
                .build();

        telemetry.setMsTransmissionInterval(100);
    }

    @Override
    public void loop() {
        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        reportBestBlob("RED", redLocator);
        reportBestBlob("BLUE", blueLocator);
        reportBestBlob("YELLOW", yellowLocator);
        telemetry.update();
    }

    // This routine drives the robot field relative
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches


        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));


        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }


    void applyBallFilters(ColorBlobLocatorProcessor locator) {
        locator.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, MIN_BALL_AREA, MAX_BALL_AREA));
        locator.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, MIN_CIRCULARITY, 1));
        locator.setSort(new ColorBlobLocatorProcessor.BlobSort(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING));
    }

    // Reports the biggest ball-shaped blob of this color, if any.
    void reportBestBlob(String colorName, ColorBlobLocatorProcessor locator) {
        List<ColorBlobLocatorProcessor.Blob> blobs = locator.getBlobs();

        if (blobs.isEmpty()) {
            telemetry.addData(colorName, "not seen");
            return;
        }

        ColorBlobLocatorProcessor.Blob best = blobs.get(0);
        Circle circle = best.getCircle();
        telemetry.addData(colorName, "x=%d y=%d radius=%d area=%d",
                (int) circle.getX(), (int) circle.getY(), (int) circle.getRadius(), best.getContourArea());
    }
}
