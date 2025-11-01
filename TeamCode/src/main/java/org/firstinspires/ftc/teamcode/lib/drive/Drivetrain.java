package org.firstinspires.ftc.teamcode.lib.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface Drivetrain {
    void set(Translation2d pose, double rt);
    void set(double strafeSpeed, double forwardSpeed, double turnSpeed, Rotation2d rt);
}