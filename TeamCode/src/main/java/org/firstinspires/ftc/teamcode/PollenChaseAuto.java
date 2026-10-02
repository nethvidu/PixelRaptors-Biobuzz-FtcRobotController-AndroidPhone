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

import java.util.ArrayList;
import java.util.List;


@Autonomous(name = "PollenChaseAuto", group = "Robot")
public class PollenChaseAuto extends LinearOpMode {
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;


    static final boolean TARGETING_BOX = false;
    static final double YELLOW_BALL_DIAMETER_CM = 2.8 * 2.54;  // ~7.1 cm
    static final double RED_BLUE_BALL_DIAMETER_CM = 3.6 * 2.54;  // ~9.1 cm
    static final double BOX_WIDTH_CM = 6.5;
    static final double BOX_HEIGHT_CM = 5.9;

    static final double BOX_DIAGONAL_CM = Math.hypot(BOX_WIDTH_CM, BOX_HEIGHT_CM);

    // Real-world width of the target, used for the distance estimate. Yellow balls are
    // smaller than red/blue ones, so each color needs its own size.
    static double targetWidthCm(String color) {
        if (TARGETING_BOX) return BOX_DIAGONAL_CM;
        return color.equals("YELLOW") ? YELLOW_BALL_DIAMETER_CM : RED_BLUE_BALL_DIAMETER_CM;
    }


    // 640x480 so a ball 144" (~366 cm) away is still ~10 px wide (a yellow ball would be
    // under 5 px at 320x240). Areas below are pixels at this resolution.
    static final int IMAGE_WIDTH = 640;
    static final int IMAGE_HEIGHT = 480;
    static final double MIN_BALL_AREA = 25;
    static final double MAX_BALL_AREA = 80000;

    static final double MIN_CIRCULARITY = TARGETING_BOX ? 0.55 : 0.5;  // tiny far-away blobs are blocky

    static final double CAMERA_FOCAL_PX = 500;  // was 250 at 320x240; scales with image width

    // The phone is mounted upright (portrait). The built-in camera image is not rotated to
    // match, so the image is turned 90 degrees: the real left/right axis is the image's Y
    // axis, not X. Set to false if the phone is mounted sideways (landscape).
    static final boolean PHONE_PORTRAIT = true;

    // ---- Driving tuning ----
    static final double DRIVE_POWER = 0.35;
    static final double TURN_SIGN = 1;
    static final double MIN_DRIVE_POWER = 0.2;
    static final double TURN_GAIN = 0.2;
    static final double MAX_TURN_POWER = 0.35;
    static final double CENTER_TOLERANCE = 0.12;
    static final double SEARCH_TURN_POWER = 0.2;
    static final double SLOW_DOWN_DISTANCE_CM = 70;
    static final long SEARCH_TIMEOUT_MS = 15000;

    // ---- Touching a ball ----
    // A ball closer than ~30 cm gets too big for the camera to see reliably (at 5 cm it would
    // not even fit in the frame), so the last stretch is driven blind: chase with vision until
    // HANDOFF_DISTANCE_CM, then drive straight for the remaining distance by time.
    static final double HANDOFF_DISTANCE_CM = 30;
    static final double TOUCH_GAP_CM = 5;           // stop this far from the ball
    static final double CAMERA_TO_FRONT_CM = 0;     // how far the phone camera sits behind the robot's front
    static final double BLIND_POWER = 0.2;
    static final double BLIND_SPEED_CM_PER_S = 15;  // TUNE: real speed at BLIND_POWER
    static final double BACKUP_CM = 25;             // back away after touching so the next ball can be seen
    static final int MAX_TOUCHES = 12;              // safety cap

    // ---- Target locking ----
    static final double TRACK_RADIUS_PX = 80;       // locked ball must stay within this of its last position
    static final long LOST_HOLD_MS = 400;           // keep going this long if the locked ball flickers out
    static final long NO_BALLS_CONFIRM_MS = 1000;   // nothing in view this long = finished

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
        long lostSince = 0;        // when the locked ball was last seen, or when nothing was in view
        String status = "searching";
        int touches = 0;

        // The ball we are currently chasing. Once picked, we keep following the same ball
        // instead of re-picking every frame, so the robot does not zig-zag between balls.
        String lockedColor = null;
        double lockedX = 0, lockedY = 0;

        while (opModeIsActive()) {
            reportBestBlob("RED", redLocator);
            reportBestBlob("BLUE", blueLocator);
            reportBestBlob("YELLOW", yellowLocator);

            List<Candidate> candidates = findCandidates();
            Candidate target = null;

            if (lockedColor != null) {
                // Follow the locked ball: same color, closest to where it was last frame.
                target = nearestToPoint(candidates, lockedColor, lockedX, lockedY);

                // A yellow ball showing up outranks a locked red/blue one.
                if (!lockedColor.equals("YELLOW")) {
                    Candidate yellow = nearestOfColor(candidates, "YELLOW");
                    if (yellow != null) {
                        target = yellow;
                    }
                }
            }

            if (target == null && lockedColor != null
                    && System.currentTimeMillis() - lostSince < LOST_HOLD_MS) {
                // Locked ball flickered out for a moment; keep the last command going.
                status = "target flickered - holding";
                telemetry.addData("Status", status);
                telemetry.update();
                continue;
            }

            if (target == null) {
                // No lock (or it was lost for good): pick the best ball in view.
                lockedColor = null;
                target = pickBest(candidates);
            }

            if (target == null) {
                if (touches == 0) {
                    // Haven't found the first ball yet: spin and look for it.
                    if (System.currentTimeMillis() - searchStart > SEARCH_TIMEOUT_MS) {
                        status = "gave up - no ball found";
                        break;
                    }
                    status = "searching";
                    drive(0, 0, SEARCH_TURN_POWER);
                } else {
                    // Already touched at least one ball and nothing else is in view: done.
                    drive(0, 0, 0);
                    if (lostSince == 0) lostSince = System.currentTimeMillis();
                    if (System.currentTimeMillis() - lostSince > NO_BALLS_CONFIRM_MS) {
                        status = "done - no more balls in view";
                        break;
                    }
                    status = "checking for more balls";
                }
            } else {
                searchStart = System.currentTimeMillis();
                lostSince = System.currentTimeMillis();
                lockedColor = target.color;
                lockedX = target.circle.getX();
                lockedY = target.circle.getY();

                double xError = steeringError(target.circle);

                telemetry.addData("chasing", "%s x=%d y=%d radius=%d",
                        target.color, (int) lockedX, (int) lockedY, (int) target.circle.getRadius());
                telemetry.addData("est. distance", "%.1f cm", target.distanceCm);
                telemetry.addData("x error", "%.2f", xError);
                telemetry.addData("touched", touches);

                if (target.distanceCm <= HANDOFF_DISTANCE_CM) {
                    touchBall(target.distanceCm);
                    touches++;
                    lockedColor = null;
                    lostSince = System.currentTimeMillis();
                    if (touches >= MAX_TOUCHES) {
                        status = "stopped - touch limit reached";
                        break;
                    }
                    continue;
                }

                double turn = TURN_SIGN * clip(xError * TURN_GAIN, MAX_TURN_POWER);
                double distScale = Math.min(1.0, (target.distanceCm - HANDOFF_DISTANCE_CM)
                        / (SLOW_DOWN_DISTANCE_CM - HANDOFF_DISTANCE_CM));
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
        telemetry.addData("balls touched", touches);
        telemetry.update();
        portal.close();

        while (opModeIsActive()) {
            idle();
        }
    }

    // One detected ball: its color, fitted circle, and estimated distance from the camera.
    static class Candidate {
        final String color;
        final Circle circle;
        final double distanceCm;

        Candidate(String color, Circle circle) {
            this.color = color;
            this.circle = circle;
            this.distanceCm = CAMERA_FOCAL_PX * targetWidthCm(color) / (circle.getRadius() * 2);
        }
    }

    // Every ball currently visible, of all three colors.
    List<Candidate> findCandidates() {
        List<Candidate> all = new ArrayList<>();
        addCandidates(all, "RED", redLocator);
        addCandidates(all, "BLUE", blueLocator);
        addCandidates(all, "YELLOW", yellowLocator);
        return all;
    }

    void addCandidates(List<Candidate> out, String color, ColorBlobLocatorProcessor locator) {
        for (ColorBlobLocatorProcessor.Blob blob : locator.getBlobs()) {
            out.add(new Candidate(color, blob.getCircle()));
        }
    }

    // Best new target: any yellow ball first (nearest yellow), otherwise the nearest red/blue.
    static Candidate pickBest(List<Candidate> candidates) {
        Candidate yellow = nearestOfColor(candidates, "YELLOW");
        if (yellow != null) return yellow;
        Candidate best = null;
        for (Candidate c : candidates) {
            if (best == null || c.distanceCm < best.distanceCm) best = c;
        }
        return best;
    }

    static Candidate nearestOfColor(List<Candidate> candidates, String color) {
        Candidate best = null;
        for (Candidate c : candidates) {
            if (c.color.equals(color) && (best == null || c.distanceCm < best.distanceCm)) best = c;
        }
        return best;
    }

    // The ball of this color closest (in the image) to the given point, if it is within
    // TRACK_RADIUS_PX. This is how the locked ball is re-found from frame to frame.
    static Candidate nearestToPoint(List<Candidate> candidates, String color, double x, double y) {
        Candidate best = null;
        double bestDist = TRACK_RADIUS_PX;
        for (Candidate c : candidates) {
            if (!c.color.equals(color)) continue;
            double d = Math.hypot(c.circle.getX() - x, c.circle.getY() - y);
            if (d <= bestDist) {
                bestDist = d;
                best = c;
            }
        }
        return best;
    }

    // -1 (target far left) .. +1 (target far right) in the robot's view.
    // Portrait: image top is the robot's right side, so a larger image Y is further left.
    // If the robot turns away from the ball, flip TURN_SIGN.
    static double steeringError(Circle circle) {
        return PHONE_PORTRAIT
                ? -(circle.getY() - IMAGE_HEIGHT / 2.0) / (IMAGE_HEIGHT / 2.0)
                : (circle.getX() - IMAGE_WIDTH / 2.0) / (IMAGE_WIDTH / 2.0);
    }

    // Drives straight in to TOUCH_GAP_CM from the ball (by time, since the ball is too close
    // for the camera), then backs away so the next ball can be seen.
    void touchBall(double distanceCm) {
        double travelCm = distanceCm - TOUCH_GAP_CM - CAMERA_TO_FRONT_CM;
        if (travelCm > 0) {
            telemetry.addData("Status", "touching ball");
            telemetry.update();
            driveFor(BLIND_POWER, (long) (travelCm / BLIND_SPEED_CM_PER_S * 1000));
        }
        drive(0, 0, 0);
        sleep(300);
        driveFor(-BLIND_POWER, (long) (BACKUP_CM / BLIND_SPEED_CM_PER_S * 1000));
        drive(0, 0, 0);
    }

    void driveFor(double forward, long ms) {
        long end = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            drive(forward, 0, 0);
            idle();
        }
        drive(0, 0, 0);
    }

    ColorBlobLocatorProcessor buildLocator(ColorRange range, int drawColor) {
        ColorBlobLocatorProcessor locator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(range)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(drawColor)
                .setBlurSize(3)  // less blur so small, distant balls survive
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
