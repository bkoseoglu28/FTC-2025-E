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
    LLResult results;
    Pose2d cameratofieldPose = new Pose2d();
    public Pose2d robotPose;
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

    Translation2d camera_to_turret = new Translation2d(0.16,0.0);
    Translation2d turret_to_robot = new Translation2d(0.11,0.0);

    @Override
    public void init(HardwareMap hardwareMap) {
        LL3 = hardwareMap.get(Limelight3A.class,"limelight");
        LL3.pipelineSwitch(0);
        LL3.start();
        filter = new LinearFilter(3);
    }

    @Override
    public void periodic() {
        if(robotPose!=null){
            Superstructure.drivetrain.poseEstimator.addVisionMeasurement(robotPose, System.currentTimeMillis(),VecBuilder.fill(results.getStddevMt1()[0],results.getStddevMt1()[1],results.getStddevMt1()[2]));
        }

    }

    @Override
    public void read() {
        this.results=LL3.getLatestResult();
        if(results!=null){

            this.tx = filter.calculate(results.getTx());

            this.ty= results.getTy();
            this.tv =results.isValid();
            if(!results.getFiducialResults().isEmpty()){
                double currentTurretAngle = Superstructure.turret.getTurretAngle();
                Rotation2d turretRot = Rotation2d.fromDegrees(currentTurretAngle);
                Translation2d rotatedLimeOffset= camera_to_turret.rotateBy(turretRot);
                Translation2d cameraRobotOfset = turret_to_robot.plus(rotatedLimeOffset);

                //Position pose = new Position();

                this.cameratofieldPose= new Pose2d(new Translation2d(
                        results.getFiducialResults().get(0).getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.METER).x,
                        results.getFiducialResults().get(0).getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.METER).y),
                        new Rotation2d(results.getFiducialResults().get(0).getCameraPoseTargetSpace().getOrientation().getYaw(AngleUnit.RADIANS)));
                this.ApriltagID=results.getFiducialResults().get(0).getFiducialId();

                Rotation2d cameratorobotcenterRot = turretRot.unaryMinus();
                Transform2d cameratorobottrans = new Transform2d(cameraRobotOfset,cameratorobotcenterRot);

                Pose2d tagPose = FieldAprilTags.TAG_20.toPose2d();
                Transform2d tagtoCamera = new Transform2d(this.cameratofieldPose.getTranslation(),this.cameratofieldPose.getRotation());

                Rotation2d correctRot = tagtoCamera.getRotation().plus(turretRot.unaryMinus());

                this.robotPose = tagPose.transformBy(tagtoCamera).transformBy(cameratorobottrans);
                //this.robotPose=getRobotToField(cameratofieldPose,FieldAprilTags.TAG_20,new Rotation2d(Units.degreesToRadians(Superstructure.turret.getTurretAngle())),Superstructure.drivetrain.OdometryModule.getRotation2d());
                //this.robotPose=getRobotPoseFromTurretCameraPose(cameratofieldPose,FieldAprilTags.TAG_20,new Rotation2d(Units.degreesToRadians(Superstructure.turret.getTurretAngle())));

            }
            if(ApriltagID==21){
                this.currentObelisk=Obelisk.GPP;
            } else if (ApriltagID==22) {
                this.currentObelisk=Obelisk.PGP;
            }else if (ApriltagID==23){
                this.currentObelisk=Obelisk.PPG;
            }
        }
    }
    public Pose2d getRobotToField(Pose2d visionPose, Pose3d tag, Rotation2d turretAngle, Rotation2d robotOrientation) {

        // Remove the inherit rotation in the apriltag pose.
        Pose2d tag2d = new Pose2d(tag.toPose2d().getTranslation(), new Rotation2d());
        Rotation2d totalAngle = visionPose.getRotation().plus(turretAngle).plus(robotOrientation);

        // visionPose is the pose of the camera relative to the tag.
        Pose2d tagToCamera = new Pose2d(visionPose.getTranslation(), new Rotation2d())
                .rotateBy(totalAngle);
                //.rotateBy(Rotation2d.fromDegrees(180));

        Transform2d tagToCameraTransform = new Transform2d(tagToCamera.getTranslation(), new Rotation2d());
        Pose2d cameraToTag = tag2d.plus(tagToCameraTransform);

        // Calculate the root relative offset of camera.
        Translation2d turretToCamera = new Translation2d(162.77/1000, 0);
        Translation2d robotToTurret = new Translation2d(-65.0/1000, 0);
        Translation2d robotToCamera = robotToTurret.plus(turretToCamera.rotateBy(turretAngle.plus(visionPose.getRotation())));

        Translation2d robotToTagTranslation = robotToCamera
                .rotateBy(robotOrientation)//.rotateBy(Rotation2d.fromDegrees(180)))
                .plus(cameraToTag.getTranslation());

        return new Pose2d(robotToTagTranslation, robotOrientation);
    }
    protected synchronized Pose2d getRobotPoseFromCameraPose(Pose2d cameraPose, Pose2d odometryPose, Rotation2d turretAngle, Pose3d tag) {

        Pose3d tagCoords = tag;
        Rotation2d fieldRelativeRobotOrientation = odometryPose.getRotation();
        Rotation2d totalCameraAngle =
                cameraPose.getRotation().plus(turretAngle).plus(fieldRelativeRobotOrientation);

        Rotation2d aprilTagToCameraLensAngle = totalCameraAngle.minus(Rotation2d.fromDegrees(0));


        double visionHorizontalDistance =tag.toPose2d().getTranslation().getDistance(cameraPose.getTranslation());



        double fieldRelativeAprilTagToCameraX = visionHorizontalDistance * aprilTagToCameraLensAngle.getCos();
        double fieldRelativeAprilTagToCameraY = visionHorizontalDistance * aprilTagToCameraLensAngle.getSin();

        double radiusFromCameraLensToTurretCenterMeters = 162.77/1000;

        // Field Oriented Limelight Rotation
        Rotation2d fieldRelativeLimelightRotation =
                Rotation2d.fromDegrees(0).minus(totalCameraAngle.plus(fieldRelativeRobotOrientation));

        double fieldRelativeCameraToTurretCenterX =
                radiusFromCameraLensToTurretCenterMeters * fieldRelativeLimelightRotation.getCos();
        double fieldRelativeCameraToTurretCenterY =
                radiusFromCameraLensToTurretCenterMeters * fieldRelativeLimelightRotation.getSin();


        double turretCenterOffsetOnRobot = -65.0/1000;

        double fieldRelativeTurretCenterToRobotCenterX =
                turretCenterOffsetOnRobot * fieldRelativeRobotOrientation.getCos();
        double fieldRelativeTurretCenterToRobotCenterY =
                turretCenterOffsetOnRobot * fieldRelativeRobotOrientation.getSin();


        double fieldRelativeDrivebaseCenterToAprilTagX = fieldRelativeAprilTagToCameraX
                + fieldRelativeCameraToTurretCenterX
                + fieldRelativeTurretCenterToRobotCenterX;
        double fieldRelativeDrivebaseCenterToAprilTagY = fieldRelativeAprilTagToCameraY
                + fieldRelativeCameraToTurretCenterY
                + fieldRelativeTurretCenterToRobotCenterY;


        double fieldRelativeRobotX = tagCoords.getX() + fieldRelativeDrivebaseCenterToAprilTagX;
        double fieldRelativeRobotY = tagCoords.getY() + fieldRelativeDrivebaseCenterToAprilTagY;


        return new Pose2d(fieldRelativeRobotX, fieldRelativeRobotY, fieldRelativeRobotOrientation);
    }
    protected synchronized Pose2d getRobotPoseFromTurretCameraPose(Pose2d visionPose, Pose3d tag, Rotation2d turretAngle){
        Translation2d turretToCamera = new Translation2d(162.77/1000, 0);
        Translation2d robotToTurret = new Translation2d(-65.0/1000, 0);
        Rotation2d robotanglecamera = visionPose.getRotation().minus(turretAngle);
        Translation2d robotTrans=visionPose.getTranslation().rotateBy(robotanglecamera).minus(turretToCamera).minus(robotToTurret);

        Pose2d robotfromapriltag=new Pose2d(robotTrans,robotanglecamera);

        Pose2d robotfromfield=robotfromapriltag.relativeTo(tag.toPose2d());
        return robotfromfield;
    }
    @Override
    public void write() {

    }

    @Override
    public void reset() {
    }
}
