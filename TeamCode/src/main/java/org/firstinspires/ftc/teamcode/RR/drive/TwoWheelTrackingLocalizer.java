package org.firstinspires.ftc.teamcode.RR.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RR.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {

    public static double CM_PER_INCH = 2.54;
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 24/25.4; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = /*1.73006*/ 13.789258 / CM_PER_INCH; // X is the up and down direction
    public static double PARALLEL_Y = /*6.77719*/ 9.8 / CM_PER_INCH; // Y is the strafe direction

    public static double PERPENDICULAR_X = /*-0.92520*/ 10.49869 / CM_PER_INCH;
    public static double PERPENDICULAR_Y = /*6.56961*/ 5.259953 / CM_PER_INCH;

    public static double MULTIPLIER_X = 1.01173585;//1.00307714;//1.002360252178778
    public static double MULTIPLIER_Y = 1.01166222;//1.0007285;//1.0067506621132265, 1.0007285

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    //imu_sensor imu;

    SampleMecanumDrive drive;
    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, /*imu_sensor imu*/ SampleMecanumDrive drive) {
        super(Arrays.asList(
            new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
            new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "MotorLR"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "MotorLF"));
        parallelEncoder.setDirection(Encoder.Direction.FORWARD);
        perpendicularEncoder.setDirection(Encoder.Direction.FORWARD);

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        //return imu.getAngle();
        return drive.getRawExternalHeading();    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * MULTIPLIER_X,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition() * MULTIPLIER_Y)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getRawVelocity() * MULTIPLIER_X),
                encoderTicksToInches(perpendicularEncoder.getRawVelocity() * MULTIPLIER_Y)
        );
    }
}