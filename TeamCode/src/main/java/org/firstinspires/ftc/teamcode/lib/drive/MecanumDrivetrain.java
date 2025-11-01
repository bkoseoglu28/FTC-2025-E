package org.firstinspires.ftc.teamcode.lib.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PinPointPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.GoBildaPinpointDriver;

public class MecanumDrivetrain extends WSubsystem implements Drivetrain {
    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackLeftMotor;
    public DcMotorEx dtBackRightMotor;
    public PinPointPoseEstimator poseEstimator;
    public GoBildaPinpointDriver OdometryModule;

    double[] pws = new double[4];
    double[] ws = new double[4];

    @Override
    public void set(double strafeSpeed, double forwardSpeed, double turnSpeed, Rotation2d rt) {
        Translation2d input = new Translation2d(strafeSpeed, forwardSpeed).rotateBy(rt.unaryMinus());

        strafeSpeed = Range.clip(input.getX(), -1, 1);
        forwardSpeed = Range.clip(input.getY(), -1, 1);
        turnSpeed = Range.clip(turnSpeed, -1, 1);

        double[] wheelSpeeds = new double[4];

        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] = forwardSpeed + strafeSpeed + turnSpeed;
        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] = forwardSpeed - strafeSpeed - turnSpeed;
        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] = (forwardSpeed - strafeSpeed + turnSpeed);
        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] = (forwardSpeed + strafeSpeed - turnSpeed);
        // 1.06, 1.04

//        if (Globals.IS_AUTO) {
//            // feedforward & voltage comp
//            double correction = 12 / robot.getVoltage();
//            for (int i = 0; i < wheelSpeeds.length; i++) {
//                wheelSpeeds[i] = Math.abs(wheelSpeeds[i]) < 0.01 ?
//                        wheelSpeeds[i] * correction :
//                        (wheelSpeeds[i] + Math.signum(wheelSpeeds[i]) * 0.085) * correction;
//            }
//
//        }

        double max = 1;
        for (double wheelSpeed : wheelSpeeds) max = Math.max(max, Math.abs(wheelSpeed));


        if (max > 1) {
            wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kBackRight.value] /= max;
        }


        ws[0] = wheelSpeeds[0];
        ws[1] = wheelSpeeds[1];
        ws[2] = wheelSpeeds[2];
        ws[3] = wheelSpeeds[3];

    }
    @Override
    public void set(Translation2d pose, double rt) {
        set(pose.getX(), pose.getY(), rt,new Rotation2d(OdometryModule.getHeading(AngleUnit.RADIANS)));
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        this.OdometryModule= hardwareMap.get(GoBildaPinpointDriver.class,"LocalizerModule");
        OdometryModule.initialize();
        OdometryModule.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        OdometryModule.recalibrateIMU();
        OdometryModule.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        OdometryModule.setOffsets(-159.5,27, DistanceUnit.MM);


        this.dtBackLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.dtFrontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtFrontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.dtBackRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.dtFrontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtFrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        poseEstimator=new PinPointPoseEstimator(OdometryModule, VecBuilder.fill(0.3,0.3,0.3),VecBuilder.fill(0.1,0.1,0.1));


    }

    @Override
    public void periodic() {
        poseEstimator.update();
        OdometryModule.update();
        // Nothing here
    }

    @Override
    public void read() {
        // Nothing here
    }
    public void setNewPose(Pose2d ps){
        OdometryModule.resetPose(ps);
    }

    @Override
    public void write() {
        if (Math.abs(ws[0] - pws[0]) > 0.005) {
            dtFrontLeftMotor.setPower(ws[0]);
            pws[0] = ws[0];
        }
        if (Math.abs(ws[1] - pws[1]) > 0.005) {
            dtFrontRightMotor.setPower(ws[1]);
            pws[1] = ws[1];
        }
        if (Math.abs(ws[2] - pws[2]) > 0.005) {
            dtBackLeftMotor.setPower(ws[2]);
            pws[2] = ws[2];
        }
        if (Math.abs(ws[3] - pws[3]) > 0.005) {
            dtBackRightMotor.setPower(ws[3]);
            pws[3] = ws[3];
        }
    }


    @Override
    public void reset() {
        OdometryModule.resetPosAndIMU();
//        if(Superstructure.vision.robotPose!=null){
//            OdometryModule.resetPose(Superstructure.vision.robotPose);
//        }else{
//            OdometryModule.resetPose(new Pose2d(new Translation2d(1.5,1.5),new Rotation2d()));
//        }
        OdometryModule.recalibrateIMU();
    }

    public String toString() {
        return "WS0: " + ws[0] + "WS1: " + ws[1] + "WS2: " + ws[2] + "WS3: " + ws[3];

    }
}
