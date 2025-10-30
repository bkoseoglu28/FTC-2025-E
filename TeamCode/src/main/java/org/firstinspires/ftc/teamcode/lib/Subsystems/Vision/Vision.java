package org.firstinspires.ftc.teamcode.lib.Subsystems.Vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.lib.math.LinearFilter;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PinPointPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Vision extends WSubsystem {
    public Limelight3A LL3;
    public double tx;
    public double ty;
    public boolean tv;
    public double yawRadians;
    public double pitchRadians;

    LLResult results;
    Pose2d cameratofieldPose = new Pose2d();
    public Pose2d RobotPose = new Pose2d();

    public Pose2d cameraToAprilTagPose;
    public Obelisk currentObelisk=Obelisk.NULL;
    int ApriltagID;
    LinearFilter filter ;
    double horizontalDistance=0.0;
    public enum Obelisk{
        PPG,
        PGP,
        GPP,
        NULL
    }

    Translation2d camera_to_turret = new Translation2d(0.16277,0);
    Translation2d turret_to_robot = new Translation2d(-0.065,0);

    @Override
    public void init(HardwareMap hardwareMap) {
        LL3 = hardwareMap.get(Limelight3A.class,"limelight");
        LL3.pipelineSwitch(0);
        LL3.start();
        filter = new LinearFilter(3);
    }

    @Override
    public void periodic() {
        if(cameraToAprilTagPose!=null){
            Superstructure.drivetrain.poseEstimator.addVisionMeasurement(cameraToAprilTagPose, System.currentTimeMillis(),VecBuilder.fill(results.getStddevMt1()[0],results.getStddevMt1()[1],results.getStddevMt1()[2]));
        }

    }

    @Override
    public void read() {
        this.results=LL3.getLatestResult();
        if(results!=null){

//            this.tx = filter.calculate(results.getTx());
            this.tx=results.getTx();
            this.ty= results.getTy();
            this.tv =results.isValid();
            yawRadians = -Units.degreesToRadians(this.tx);
            pitchRadians = Units.degreesToRadians(this.ty);
            Rotation2d turretangle= Rotation2d.fromDegrees(Superstructure.turret.getTurretAngle());
            double theta = pitchRadians + Units.degreesToRadians(24.62);

            double heightFromRobotToTargetMeters = 0.7493 - 0.31777;
            double horizontalDistanceToTargetMeters = heightFromRobotToTargetMeters / Math.tan(theta);
            cameraToAprilTagPose = new Pose2d(
                    horizontalDistanceToTargetMeters * Math.cos(yawRadians),
                    horizontalDistanceToTargetMeters * Math.sin(yawRadians),
                    Rotation2d.fromRadians(yawRadians));

            this.RobotPose=getRobotToField(cameratofieldPose,FieldAprilTags.TAG_20,turretangle,Superstructure.drivetrain.OdometryModule.getRotation2d());
            if(ApriltagID==21){
                this.currentObelisk=Obelisk.GPP;
            } else if (ApriltagID==22) {
                this.currentObelisk=Obelisk.PGP;
            }else if (ApriltagID==23){
                this.currentObelisk=Obelisk.PPG;
            }
        }
    }

    public static Pose2d toPose2D(Pose3d pose3d){
        return new Pose2d(new Translation2d(pose3d.getX(),pose3d.getY()),
                new Rotation2d(pose3d.getRotation().getZ()));
    }
    public static Transform2d toTransform2D(Pose2d pose2d){
        return new Transform2d(pose2d.getTranslation(),pose2d.getRotation());
    }
    public static Transform2d transformByy(Transform2d transA, Transform2d transB){
        Translation2d newTrans = transA.getTranslation().plus(transB.getTranslation().rotateBy(transA.getRotation()));
        Rotation2d newRot = transA.getRotation().plus(transB.getRotation());
        return new Transform2d(newTrans,newRot);
    }

    public static Transform2d inverse(Transform2d transform){
        Rotation2d inversRot = transform.getRotation().unaryMinus();

        Translation2d inversTrans = transform.getTranslation().unaryMinus().rotateBy(inversRot);

        return new Transform2d(inversTrans,inversRot);

    }

    protected Pose2d getRobotToField(
            Pose2d visionPose, Pose3d tag, Rotation2d turretAngle, Rotation2d robotOrientation) {

        // Remove the inherit rotation in the apriltag pose.
        Pose2d tag2d = tag.toPose2d();
        Rotation2d totalAngle = visionPose.getRotation().plus(turretAngle).plus(robotOrientation);

        // visionPose is the pose of the camera relative to the tag.
        Pose2d tagToCamera = new Pose2d(visionPose.getTranslation(), new Rotation2d())
                .rotateBy(totalAngle)
                .rotateBy(Rotation2d.fromDegrees(0));

        Transform2d tagToCameraTransform = new Transform2d(tagToCamera.getTranslation(), new Rotation2d());
        Pose2d cameraToTag = tag2d.plus(tagToCameraTransform);

        // Calculate the root relative offset of camera.
        Translation2d turretToCamera = new Translation2d(0.16277,0);
        Translation2d robotToTurret = new Translation2d(-0.065,0);
        Translation2d robotToCamera = robotToTurret.plus(turretToCamera.rotateBy(turretAngle.plus(visionPose.getRotation())));

        Translation2d robotToTagTranslation = robotToCamera
                .rotateBy(robotOrientation.rotateBy(Rotation2d.fromDegrees(0)))
                .plus(cameraToTag.getTranslation());

        return new Pose2d(robotToTagTranslation, robotOrientation);
    }
    @Override
    public void write() {

    }

    @Override
    public void reset() {
    }
}
