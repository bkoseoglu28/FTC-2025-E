package org.firstinspires.ftc.teamcode.lib.Subsystems.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.lib.math.LinearFilter;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Vision extends WSubsystem {
    public Limelight3A LL3;
    public double tx;
    public double ty;
    public boolean tv;
    public double yawRadians;
    public double pitchRadians;

    LLResult results;
    Pose2d TargetCamera = new Pose2d();
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
        if(Superstructure.IsBlue){
            LL3.pipelineSwitch(0);
        }else{
            LL3.pipelineSwitch(1);
        }
        LL3.start();
        filter = new LinearFilter(3);
    }

    @Override
    public void periodic() {
//        if(cameraToAprilTagPose!=null){
//            Superstructure.drivetrain.poseEstimator.addVisionMeasurement(cameraToAprilTagPose, System.currentTimeMillis(),VecBuilder.fill(results.getStddevMt1()[0],results.getStddevMt1()[1],results.getStddevMt1()[2]));
//        }

    }

    @Override
    public void read() {
        this.results=LL3.getLatestResult();
        if(results!=null){

//            this.tx = filter.calculate(results.getTx());
            this.tx=results.getTx();
            this.ty= results.getTy();
            this.tv =results.isValid();
            if(this.tv){
                LLResultTypes.FiducialResult tag =results.getFiducialResults().get(0);
                if(tag.getFiducialId()==20){
                    Pose3D TargetPoseCamera =tag.getTargetPoseCameraSpace();
                    TargetCamera = new Pose2d(new Translation2d(TargetPoseCamera.getPosition().toUnit(DistanceUnit.METER).x, TargetPoseCamera.getPosition().toUnit(DistanceUnit.METER).y),new Rotation2d(TargetPoseCamera.getOrientation().getYaw(AngleUnit.RADIANS)));
                    this.RobotPose=getRobotToField(TargetCamera,FieldAprilTags.TAG_20,Rotation2d.fromDegrees(Superstructure.turret.getTurretAngle()),Rotation2d.fromDegrees(90));
                }
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
        Pose2d tag2d = new Pose2d(tag.toPose2d().getTranslation(), new Rotation2d());
        Rotation2d totalAngle = visionPose.getRotation().plus(turretAngle).plus(robotOrientation);

        // visionPose is the pose of the camera relative to the tag.
        Pose2d tagToCamera = new Pose2d(visionPose.getTranslation(), new Rotation2d())
                .rotateBy(totalAngle);

        Transform2d tagToCameraTransform = new Transform2d(tagToCamera.getTranslation(), new Rotation2d());
        Pose2d cameraToTag = tag2d.plus(tagToCameraTransform);

        // Calculate the root relative offset of camera.
        Translation2d turretToCamera = new Translation2d(0.16277, 0);
        Translation2d robotToTurret = new Translation2d(-0.065, 0);
        Translation2d robotToCamera = robotToTurret.plus(turretToCamera.rotateBy(turretAngle.plus(visionPose.getRotation())));

        Translation2d robotToTagTranslation = robotToCamera
                .rotateBy(robotOrientation)
                .plus(cameraToTag.getTranslation());

        return new Pose2d(robotToTagTranslation, robotOrientation);
    }
    protected Pose2d getDeepSeekRobotToField(
            Pose2d visionPose, Pose3d tag, Rotation2d turretAngle, Rotation2d robotOrientation) {

        Pose2d tagOnField = tag.toPose2d();

        // Offsets
        Translation2d cameraToTurret = new Translation2d(0.16277, 0);
        Translation2d turretToRobot = new Translation2d(-0.065, 0);

        // Step 1: Camera in field coordinates
        Pose2d cameraOnField = tagOnField.transformBy(new Transform2d(
                visionPose.getTranslation().rotateBy(tagOnField.getRotation()),
                visionPose.getRotation()
        ));

        // Step 2: Calculate robot orientation
        // camera field rotation = robot orientation + turret angle + camera relative to turret
        // So: robot orientation = camera field rotation - turret angle - camera relative to turret
        Rotation2d calculatedRobotOrientation = cameraOnField.getRotation()
                .minus(turretAngle);

        // Step 3: Calculate robot position using transformation chain
        // Camera -> Turret -> Robot
        Transform2d cameraToTurretTransform = new Transform2d(
                cameraToTurret.rotateBy(visionPose.getRotation()),
                new Rotation2d()
        );

        Transform2d turretToRobotTransform = new Transform2d(
                turretToRobot.rotateBy(calculatedRobotOrientation),
                new Rotation2d()
        );

        // Combine transformations: camera -> turret -> robot
        Pose2d robotOnField = cameraOnField
                .transformBy(cameraToTurretTransform)
                .transformBy(turretToRobotTransform);

        return new Pose2d(robotOnField.getTranslation(), calculatedRobotOrientation);
    }
    @Override
    public void write() {

    }

    @Override
    public void reset() {
    }
}
