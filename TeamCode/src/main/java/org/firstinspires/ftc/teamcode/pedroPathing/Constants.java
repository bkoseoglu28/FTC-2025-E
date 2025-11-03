package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
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

import edu.wpi.first.math.util.Units;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(20.0)
            .forwardZeroPowerAcceleration(-25.9346931313679598)
            .lateralZeroPowerAcceleration(-67.342491844080064)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.1,
                    0.0,
                    0.01,
                    0.0
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.1,
                    0.0,
                    0.01,
                    0.0
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    2.0,
                    0,
                    0.1,
                    0.0
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2.0,
                    0.0,
                    0.1,
                    0.0
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.005,
                    0.0,
                    0.0015,
                    0.6,
                    0.0
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.005,
                    0.0,
                    0.0015,
                    0.6,
                    0.0
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0009);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("frontLeftMotor")
            .leftRearMotorName("backLeftMotor")
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(62.5)
            .yVelocity(53.4);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(DistanceUnit.INCH.fromMm(-22.99))
            .strafePodX(DistanceUnit.INCH.fromMm(-163.51))
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("LocalizerModule")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    /**
     These are the PathConstraints in order:
     tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
     brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart

     The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
     */

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.007,
            500,
            1.25,
            10,
            1
    );

    //Add custom localizers or drivetrains here
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }

}