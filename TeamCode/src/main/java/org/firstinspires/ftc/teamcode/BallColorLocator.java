package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

/*
 * This OpMode uses the phone's built-in camera to find the red, blue, and yellow BioBuzz balls.
 * It runs one ColorBlobLocatorProcessor per color (they all share the same camera feed) and
 * reports the biggest, most-circular blob found for each color: its pixel position and radius.
 *
 * Balls are ~3.6" (red/blue) and ~2.8" (yellow) in diameter. The blob's pixel radius will grow
 * as a ball gets closer to the camera - use that, along with the known real-world size, if you
 * need to estimate distance later.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new
 * name if you want to customize the color ranges or filtering for your field lighting.
 */
@TeleOp(name = "BallColorLocator", group = "Robot")
public class BallColorLocator extends LinearOpMode {

    // Minimum/maximum contour area (in pixels) a blob must have to be considered a ball,
    // at 320x240 resolution. Tune these once you see real ball sizes in the telemetry.
    static final double MIN_BALL_AREA = 150;
    static final double MAX_BALL_AREA = 20000;
    static final double MIN_CIRCULARITY = 0.7;

    ColorBlobLocatorProcessor redLocator;
    ColorBlobLocatorProcessor blueLocator;
    ColorBlobLocatorProcessor yellowLocator;

    @Override
    public void runOpMode() {
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
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // Run during INIT too, so you can see the camera preview and tune before starting.
        while (opModeIsActive() || opModeInInit()) {
            reportBestBlob("RED", redLocator);
            reportBestBlob("BLUE", blueLocator);
            reportBestBlob("YELLOW", yellowLocator);
            telemetry.update();
            sleep(100);
        }
    }

    // Tells the processor itself to drop non-ball blobs, so both getBlobs() and the drawn
    // camera preview only ever show blobs that pass these criteria - not just our telemetry.
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
