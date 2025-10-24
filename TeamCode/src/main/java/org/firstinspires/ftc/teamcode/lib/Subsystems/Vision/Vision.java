package org.firstinspires.ftc.teamcode.lib.Subsystems.Vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
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
    public enum Obelisk{
        PPG,
        PGP,
        GPP,
        NULL
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        LL3 = hardwareMap.get(Limelight3A.class,"limelight");
        LL3.pipelineSwitch(0);
        LL3.start();
    }

    @Override
    public void periodic() {
        if(robotPose!=null){
            Superstructure.drivetrain.poseEstimator.addVisionMeasurement(robotPose, System.currentTimeMillis());
        }

    }

    @Override
    public void read() {
        this.results=LL3.getLatestResult();
        if(results!=null){
            this.tx = results.getTx();
            this.ty= results.getTy();
            this.tv =results.isValid();
            if(!results.getFiducialResults().isEmpty()){
                this.cameratofieldPose= new Pose2d(new Translation2d(
                        results.getFiducialResults().get(0).getCameraPoseTargetSpace().getPosition().x,
                        results.getFiducialResults().get(0).getCameraPoseTargetSpace().getPosition().y),
                        new Rotation2d(results.getFiducialResults().get(0).getCameraPoseTargetSpace().getOrientation().getYaw(AngleUnit.RADIANS)));
                this.ApriltagID=results.getFiducialResults().get(0).getFiducialId();
                this.robotPose=getRobotToField(cameratofieldPose,FieldAprilTags.TAG_24,new Rotation2d(Units.degreesToRadians(Superstructure.turret.getTurretAngle())),Superstructure.drivetrain.OdometryModule.getRotation2d());
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
    public Pose2d getRobotToField(
            Pose2d visionPose, Pose3d tag, Rotation2d turretAngle, Rotation2d robotOrientation) {

        // Remove the inherit rotation in the apriltag pose.
        Pose2d tag2d = new Pose2d(tag.toPose2d().getTranslation(), new Rotation2d());
        Rotation2d totalAngle = visionPose.getRotation().plus(turretAngle).plus(robotOrientation);

        // visionPose is the pose of the camera relative to the tag.
        Pose2d tagToCamera = new Pose2d(visionPose.getTranslation(), new Rotation2d())
                .rotateBy(totalAngle)
                .rotateBy(Rotation2d.fromDegrees(180));

        Transform2d tagToCameraTransform = new Transform2d(tagToCamera.getTranslation(), new Rotation2d());
        Pose2d cameraToTag = tag2d.plus(tagToCameraTransform);

        // Calculate the root relative offset of camera.
        Translation2d turretToCamera = new Translation2d(162.77/1000, 0);
        Translation2d robotToTurret = new Translation2d(65.0/1000, 0);
        Translation2d robotToCamera = robotToTurret.plus(turretToCamera.rotateBy(turretAngle.plus(visionPose.getRotation())));

        Translation2d robotToTagTranslation = robotToCamera
                .rotateBy(robotOrientation.rotateBy(Rotation2d.fromDegrees(180)))
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
