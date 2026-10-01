package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;


@Autonomous(name = "PollenChaseAuto", group = "Robot")
public class PollenChaseAuto extends LinearOpMode {
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;


    static final boolean TARGETING_BOX = true;
    static final double BALL_DIAMETER_CM = 2.8 * 2.54;  // yellow ball, ~7.1 cm
    static final double BOX_WIDTH_CM = 6.5;
    static final double BOX_HEIGHT_CM = 5.9;

    static final double TARGET_WIDTH_CM = TARGETING_BOX
            ? Math.hypot(BOX_WIDTH_CM, BOX_HEIGHT_CM)
            : BALL_DIAMETER_CM;


    static final int IMAGE_WIDTH = 320;
    static final int IMAGE_HEIGHT = 240;
    static final double MIN_BALL_AREA = 150;
    static final double MAX_BALL_AREA = 20000;

    static final double MIN_CIRCULARITY = TARGETING_BOX ? 0.55 : 0.7;

    static final double CAMERA_FOCAL_PX = 250;

    // ---- Driving tuning ----
    static final double STOP_DISTANCE_CM = 30;
    static final double DRIVE_POWER = 0.35;
    static final double TURN_SIGN = 1;
    static final double MIN_DRIVE_POWER = 0.2;
    static final double TURN_GAIN = 0.2;
    static final double MAX_TURN_POWER = 0.35;
    static final double CENTER_TOLERANCE = 0.12;
    static final double SEARCH_TURN_POWER = 0.2;
    static final double SLOW_DOWN_DISTANCE_CM = 40;
    static final long SEARCH_TIMEOUT_MS = 15000;

    ColorBlobLocatorProcessor redLocator;
    ColorBlobLocatorProcessor blueLocator;
    ColorBlobLocatorProcessor yellowLocator;
    VisionPortal portal;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        redLocator = buildLocator(ColorRange.RED, Color.RED);
        blueLocator = buildLocator(ColorRange.BLUE, Color.BLUE);
        yellowLocator = buildLocator(ColorRange.YELLOW, Color.YELLOW);

        portal = new VisionPortal.Builder()
                .addProcessor(redLocator)
                .addProcessor(blueLocator)
                .addProcessor(yellowLocator)
                .setCameraResolution(new Size(IMAGE_WIDTH, IMAGE_HEIGHT))
                .setCamera(BuiltinCameraDirection.BACK)
                .build();

        telemetry.setMsTransmissionInterval(100);
        telemetry.addData("Target", TARGETING_BOX ? "red/blue/yellow BOX" : "red/blue/yellow BALL");
        telemetry.addData("Status", "Ready - press START");
        telemetry.update();

        waitForStart();

        long searchStart = System.currentTimeMillis();
        String status = "searching";

        while (opModeIsActive()) {
            reportBestBlob("RED", redLocator);
            reportBestBlob("BLUE", blueLocator);
            reportBestBlob("YELLOW", yellowLocator);

            // Chase whichever color currently has the biggest (closest) blob.
            String targetColor = null;
            ColorBlobLocatorProcessor.Blob target = null;
            String[] names = {"RED", "BLUE", "YELLOW"};
            ColorBlobLocatorProcessor[] locators = {redLocator, blueLocator, yellowLocator};
            for (int i = 0; i < locators.length; i++) {
                List<ColorBlobLocatorProcessor.Blob> found = locators[i].getBlobs();
                if (!found.isEmpty() && (target == null
                        || found.get(0).getContourArea() > target.getContourArea())) {
                    target = found.get(0);
                    targetColor = names[i];
                }
            }

            if (target == null) {

                if (System.currentTimeMillis() - searchStart > SEARCH_TIMEOUT_MS) {
                    status = "gave up - target not found";
                    break;
                }
                status = "searching";
                drive(0, 0, SEARCH_TURN_POWER);
            } else {
                searchStart = System.currentTimeMillis();

                Circle circle = target.getCircle();
                double diameterPx = circle.getRadius() * 2;
                double distanceCm = CAMERA_FOCAL_PX * TARGET_WIDTH_CM / diameterPx;

                double xError = (circle.getX() - IMAGE_WIDTH / 2.0) / (IMAGE_WIDTH / 2.0);

                telemetry.addData("chasing", "%s x=%d y=%d radius=%d",
                        targetColor, (int) circle.getX(), (int) circle.getY(), (int) circle.getRadius());
                telemetry.addData("est. distance", "%.1f cm", distanceCm);
                telemetry.addData("x error", "%.2f", xError);

                if (distanceCm <= STOP_DISTANCE_CM) {
                    status = "arrived";
                    break;
                }

                double turn = TURN_SIGN * clip(xError * TURN_GAIN, MAX_TURN_POWER);
                double distScale = Math.min(1.0, (distanceCm - STOP_DISTANCE_CM)
                        / (SLOW_DOWN_DISTANCE_CM - STOP_DISTANCE_CM));
                double alignScale = Math.max(0.3, 1.0 - Math.abs(xError));
                double forward = Math.max(MIN_DRIVE_POWER, DRIVE_POWER * distScale * alignScale);
                status = Math.abs(xError) <= CENTER_TOLERANCE ? "driving to target" : "chasing target";
                telemetry.addData("forward / turn", "%.2f / %.2f", forward, turn);
                drive(forward, 0, turn);
            }

            telemetry.addData("Status", status);
            telemetry.update();
        }

        drive(0, 0, 0);
        telemetry.addData("Status", status);
        telemetry.update();
        portal.close();

        while (opModeIsActive()) {
            idle();
        }
    }

    ColorBlobLocatorProcessor buildLocator(ColorRange range, int drawColor) {
        ColorBlobLocatorProcessor locator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(range)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(drawColor)
                .setBlurSize(5)
                .build();
        locator.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, MIN_BALL_AREA, MAX_BALL_AREA));
        locator.addFilter(new ColorBlobLocatorProcessor.BlobFilter(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, MIN_CIRCULARITY, 1));
        locator.setSort(new ColorBlobLocatorProcessor.BlobSort(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING));
        return locator;
    }

    // Reports the biggest ball-shaped blob of this color, if any.
    void reportBestBlob(String colorName, ColorBlobLocatorProcessor locator) {
        List<ColorBlobLocatorProcessor.Blob> blobs = locator.getBlobs();
        if (blobs.isEmpty()) {
            telemetry.addData(colorName, "not seen");
            return;
        }
        Circle circle = blobs.get(0).getCircle();
        telemetry.addData(colorName, "x=%d y=%d radius=%d area=%d",
                (int) circle.getX(), (int) circle.getY(), (int) circle.getRadius(),
                blobs.get(0).getContourArea());
    }

    static double clip(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }


    void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeftDrive.setPower(frontLeftPower / maxPower);
        frontRightDrive.setPower(frontRightPower / maxPower);
        backLeftDrive.setPower(backLeftPower / maxPower);
        backRightDrive.setPower(backRightPower / maxPower);
    }
}
