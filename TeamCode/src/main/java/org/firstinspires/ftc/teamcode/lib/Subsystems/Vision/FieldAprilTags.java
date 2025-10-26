package org.firstinspires.ftc.teamcode.lib.Subsystems.Vision;

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
            new Rotation3d(0, 0, Math.toRadians(53.13-90)) // approx from rotation matrix
    );

    public static final Pose3d TAG_24 = new Pose3d(
            new Translation3d(-1.4827, 1.4133, 0.7493),
            new Rotation3d(0, 0, Math.toRadians(-53.13-90))
    );

    public static final double FIELD_LENGTH_METERS = 17.5482504;
    public static final double FIELD_WIDTH_METERS = 8.0519016;
}
