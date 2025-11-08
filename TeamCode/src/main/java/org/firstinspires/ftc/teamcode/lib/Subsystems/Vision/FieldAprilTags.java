package org.firstinspires.ftc.teamcode.lib.Subsystems.Vision;

import com.pedropathing.geometry.Pose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * FTC 2025 Field AprilTag positions (decoded from .fmap)
 * All distances are in meters.
 */
public final class FieldAprilTags {
    public static final Pose3d TAG_20 = new Pose3d(
            new Translation3d(-1.4827, -1.4133, 0.7493),
            new Rotation3d(0, 0, Math.toRadians(53.13)) // approx from rotation matrix
    );

    public static final Pose3d TAG_24 = new Pose3d(
            new Translation3d(-1.4827, 1.4133, 0.7493),
            new Rotation3d(0, 0, Math.toRadians(-53.13))
    );
    public static final Pose PEDRO_TAG_20 = toPedroPose(TAG_20);
    public static final Pose PEDRO_TAG_24 = toPedroPose(TAG_24);
    private static final double METERS_TO_INCHES = 39.3701;
    private static final double FIELD_SIZE_INCHES = 144.0;
    public static final double FIELD_LENGTH_METERS = 17.5482504;
    public static final double FIELD_WIDTH_METERS = 8.0519016;
    /**
     * Converts a WPILib Pose3d (center-origin, meters) to a Pedro Pose (bottom-left origin, inches).
     */
    private static Pose toPedroPose(Pose3d tag) {
        // Convert translation from meters to inches
        double xInches = tag.getX() * METERS_TO_INCHES;
        double yInches = tag.getY() * METERS_TO_INCHES;

        // Convert from center-origin to bottom-left origin
        double xPedro = xInches + FIELD_SIZE_INCHES / 2.0;
        double yPedro = yInches + FIELD_SIZE_INCHES / 2.0;

        // Convert heading: Pedro 0° = +X, FTC 0° = +Y → subtract 90°
        double headingPedro = tag.getRotation().getZ() - Math.toRadians(90);

        return new Pose(xPedro, yPedro, headingPedro);
    }


}
