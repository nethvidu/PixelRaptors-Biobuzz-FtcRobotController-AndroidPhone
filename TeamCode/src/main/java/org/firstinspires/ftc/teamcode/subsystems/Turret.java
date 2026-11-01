package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.TurretPIDF;

/**
 * Hardware wrapper for the turret: drive motor + OctoQuad reading a pulse-width absolute
 * encoder (channel 4, used once at init to calibrate the forward-zero) and a quadrature
 * encoder (channel 0, the running angle).
 *
 * <p>All public angles are in radians, canonical frame: 0 = robot's right, CCW positive.
 * Mechanical travel: a 404° CW arc that starts at MAX_ANGLE (108°) and ends at
 * MIN_ANGLE (-296°, equivalent to 64° mod 2π). The 44° band in canonical [64°, 108°]
 * is reachable two ways. The internal cumulative target lives in the linear interval
 * [MIN_ANGLE, MAX_ANGLE] = [-296°, 108°]; {@link #getCurrentAngle()} returns the
 * canonical angle wrapped into [0, 2π).
 */
@Config
public class Turret {
    // Device names in the Robot Configuration.
    public static final String OCTOQUAD = "octoquad";
    public static final String TURRET = "turret";

    private static final double TWO_PI = 2 * Math.PI;
    private static final double HALF_PI = Math.PI / 2;
    // goBILDA Gear-Driven Turret (2.75:1): 64T hub-mount gear drives the 176T turret ring.
    // The encoder sits on a second 64T gear meshed with the ring, so it turns 1:1 with the motor gear
    // (2.75 encoder revs per turret rev).
    private static final double SMALL_GEAR_TEETH = 64.0;
    private static final double LARGE_GEAR_TEETH = 176.0;
    private static final double GEAR_RATIO = LARGE_GEAR_TEETH / SMALL_GEAR_TEETH;
    private static final double ABS_ENCODER_RANGE = 1000.0;
    private static final double QUAD_ENCODER_RANGE = 8192.0;
    private static final double ABS_SENSOR_FORWARD_ANGLE = Math.toRadians(54.0);
    private static final int ABS_ENCODER_OFFSET = 416;
    private static final int QUAD_CHANNEL = 0;
    private static final int ABS_CHANNEL = 4;

    private static final double MIN_ANGLE = Math.toRadians(-296);
    private static final double MAX_ANGLE = Math.toRadians(108);
    private static final double D_FILTER = 0.8;
    public double angleOffset = 0;

    // Dashboard-tunable PIDF; must be tuned on th
    public static double kP = 0.03, kI = 0.007, kD = 0.00145, kF = 0.002;
    //public static double kP = 0.03, kI = 0.004, kD = 0.0008, kF = 0.0;


    private double angularVelocity = 0;
    public static double angularConstant = 0;
    private double lastHeading = Double.NaN;
    private final ElapsedTime headingTimer = new ElapsedTime();

    private final TurretPIDF pid = new TurretPIDF(kP, kI, kD, kF);
    private OctoQuad octoQuad;
    private DcMotor motor;

    private double offset = 0;
    private double targetAngle = 0;
    private double cachedAngle = 0;

    /**
     * Resume constructor — rebuilds the turret's angle frame from a value persisted before the
     * last power-off, instead of recalibrating against the pulse-width absolute encoder.
     *
     * <p>Use this <em>only</em> when the robot is powered back on without the turret having been
     * moved by hand while off: the saved position is then still physically valid. Construct with
     * this and do <strong>not</strong> call {@link #Turret(HardwareMap)} — this fully replaces it.
     *
     * <p><b>Why this avoids wrap-around.</b> {@link #Turret(HardwareMap)} derives the forward-zero
     * from the absolute encoder, which completes a full revolution every {@code 360°/GEAR_RATIO}
     * (108°) of turret output — so over the 404° mechanical travel it repeats ~3.7 times and
     * cannot tell which lap the turret is on. init() papers over that by assuming boot near
     * forward. This constructor never reads the absolute encoder; the saved cumulative angle
     * already carries the full unwrapped position, so the lap is known exactly and there is no
     * ambiguity.
     *
     * <p>The quadrature count is volatile across a power cycle, so it is zeroed here and the
     * {@link #offset} is chosen so the angle frame reproduces {@code savedRawAngle} from
     * whatever the encoder reads now. {@code cachedAngle}/{@code targetAngle} are seeded to that
     * same position so the turret holds where it powered off until {@link #setTargetAngle(double)}
     * is called.
     *
     * @param hardwareMap   robot hardware map.
     * @param savedRawAngle the raw cumulative turret angle (radians) saved before shutdown: the
     *                      value {@code getAngle()} returns / that {@code cachedAngle} holds. It
     *                      is in the raw, CW-positive, forward-zero frame the PIDF drives in and
     *                      is unwrapped (may be negative or &gt; 2π). Persist this exact value —
     *                      do NOT persist {@link #getCurrentAngle()}, which is wrapped to [0, 2π)
     *                      and loses the lap needed to resume safely.
     */
    public Turret(HardwareMap hardwareMap, double savedRawAngle) {
        octoQuad = hardwareMap.get(OctoQuad.class, OCTOQUAD);
        motor = hardwareMap.get(DcMotor.class, TURRET);

        // Quadrature is needed for the running angle; the absolute channel is deliberately left
        // unread, so its bank config is irrelevant here.
        octoQuad.setChannelBankConfig(OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);

        // Force the quad count to a known baseline (it does not survive power-off), then pick the
        // offset so getAngle() == savedRawAngle regardless of what the count reads right now.
        octoQuad.resetSinglePosition(QUAD_CHANNEL);
        OctoQuad.EncoderDataBlock encoderData = octoQuad.readAllEncoderData();
        int quadRaw = encoderData.positions[QUAD_CHANNEL];
        double quadAngleNow = convertEncoderValueToRadians(quadRaw, QUAD_ENCODER_RANGE);
        offset = quadAngleNow - savedRawAngle;

        // Hold the powered-off position until a target is explicitly set.
        cachedAngle = getAngle();
        targetAngle = cachedAngle;
    }

    /**
     * Acquires hardware, calibrates the forward-zero from the absolute encoder, and resets
     * the quadrature encoder. Also seeds the PID target to the current raw angle so the
     * turret holds position until {@link #setTargetAngle(double)} is called. Call exactly
     * once before any other method.
     *
     * @return the initial PID target in the raw encoder frame (radians); use
     *         {@link #getCurrentAngle()} if you need a canonical-frame readout.
     */
    public Turret(HardwareMap hardwareMap) {
        octoQuad = hardwareMap.get(OctoQuad.class, OCTOQUAD);
        motor = hardwareMap.get(DcMotor.class, TURRET);

        octoQuad.setChannelBankConfig(OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);
        octoQuad.setSingleChannelPulseWidthTracksWrap(ABS_CHANNEL, false);
        octoQuad.setSingleChannelPulseWidthParams(ABS_CHANNEL, 3, 1003);

        homeFromAbsoluteEncoder();
    }

    public void homeFromAbsoluteEncoder() {
        OctoQuad.EncoderDataBlock encoderData = octoQuad.readAllEncoderData();
        int absRaw = encoderData.positions[ABS_CHANNEL];
        int offsettedRaw = (int) ((absRaw - ABS_ENCODER_OFFSET + ABS_ENCODER_RANGE) % ABS_ENCODER_RANGE);
        double initialAbsAngle = convertEncoderValueToRadians(offsettedRaw, ABS_ENCODER_RANGE);
        offset = initialAbsAngle - ABS_SENSOR_FORWARD_ANGLE;

        octoQuad.resetSinglePosition(QUAD_CHANNEL);

        // Hold position until something explicitly sets a target. Seed cachedAngle from this
        // same read so getCurrentAngle()/setTargetAngle() are valid before the first update().
        cachedAngle = getAngle();
        targetAngle = cachedAngle;
    }

    /**
     * Runs one PIDF cycle and commands the motor. Call every loop. The robot heading (radians)
     * is only used for the heading feedforward (angularConstant); pass 0 if there is no localizer.
     */
    public void update(double currentHeading) {
        double dt = headingTimer.seconds();
        headingTimer.reset();
        if (!Double.isNaN(lastHeading) && dt > 0.001) {
            double headingDelta = currentHeading - lastHeading;
            while (headingDelta > Math.PI) headingDelta -= 2 * Math.PI;
            while (headingDelta < -Math.PI) headingDelta += 2 * Math.PI;
            angularVelocity = headingDelta / dt;
        }
        lastHeading = currentHeading;
        pid.setGains(kP, kI, kD, kF);
        pid.setTarget(Math.toDegrees(targetAngle + (angularVelocity * angularConstant)));
        cachedAngle = getAngle();
        motor.setPower(-pid.calculate(Math.toDegrees(cachedAngle)));
    }

    public void moveMotor(double power) {
        motor.setPower(power);
    }

    /**
     * Non-blocking. Stores the setpoint the next {@link #update(double)} will drive toward.
     * Wraps the input into a cumulative setpoint inside [MIN_ANGLE, MAX_ANGLE] and,
     * in the 44° overlap, picks the candidate closest to the current angle (shortest path).
     */
    // cachedAngle must be current before this is called — update() must run first each loop.
    public void setTargetAngle(double newAngle) {
        double currentAngle = HALF_PI - cachedAngle;

        while (newAngle < MIN_ANGLE) newAngle += TWO_PI;
        while (newAngle > MAX_ANGLE) newAngle -= TWO_PI;

        double altHigh = newAngle + TWO_PI;
        if (altHigh <= MAX_ANGLE
                && Math.abs(altHigh - currentAngle) < Math.abs(newAngle - currentAngle)) {
            newAngle = altHigh;
        }
        double altLow = newAngle - TWO_PI;
        if (altLow >= MIN_ANGLE
                && Math.abs(altLow - currentAngle) < Math.abs(newAngle - currentAngle)) {
            newAngle = altLow;
        }

        targetAngle = HALF_PI - newAngle+angleOffset;
    }

    /** @return the setpoint the PIDF is currently driving toward (radians, canonical frame). */
    public double getTargetAngle() {
        return (TWO_PI - targetAngle + HALF_PI) % TWO_PI;
    }

    /**
     * @return live turret angle (radians, canonical frame), wrapped to [0, 2π). This loses
     *         the overlap-branch information from the cumulative encoder; the PIDF uses the
     *         raw cumulative angle internally to pick the shorter path.
     *
     *         <p>Must be called only after {@link #Turret(HardwareMap)}.
     */
    public double getCurrentAngle() {
        return (TWO_PI - cachedAngle + HALF_PI) % TWO_PI;
    }

    /**
     * @return the raw cumulative turret angle (radians) — the same unwrapped, CW-positive,
     *         forward-zero value the PIDF drives in. Unlike {@link #getCurrentAngle()} this is
     *         NOT wrapped, so it carries the lap/overlap branch. Persist this across a power
     *         cycle and feed it back to {@link #Turret(HardwareMap, double)} to resume exactly.
     */
    public double getRawAngle() {
        return cachedAngle;
    }

    /**
     * Cumulative angle from the quad encoder, in the raw frame: 0 = forward (set by the
     * init calibration), positive = CW. Not normalized; can be negative or > 2π. The
     * canonical frame used by the public API is the mirror, HALF_PI - getAngle().
     */
    private double getAngle() {
        OctoQuad.EncoderDataBlock encoderData = octoQuad.readAllEncoderData();
        int quadRaw = encoderData.positions[QUAD_CHANNEL];
        return convertEncoderValueToRadians(quadRaw, QUAD_ENCODER_RANGE) - offset;
    }

    /** Raw encoder ticks → radians at the turret output (accounts for GEAR_RATIO). */
    private double convertEncoderValueToRadians(int rawEncoderValue, double encoderRange) {
        return (rawEncoderValue * TWO_PI) / (encoderRange * GEAR_RATIO);
    }
}
