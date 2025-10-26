package org.firstinspires.ftc.teamcode;

import android.telephony.IccOpenLogicalChannelResponse;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.MathUtils;
import org.firstinspires.ftc.teamcode.lib.RTPAxon;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.Sensors;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Feeder.Feeder;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Vision.FieldAprilTags;
import org.firstinspires.ftc.teamcode.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.wrappers.WVelocityGroup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.GoBildaPinpointDriver;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.util.Units;
@Config
@TeleOp(name = "SOLO")
public class SOLO extends OpMode {
    FtcDashboard dashboard;
    public static double targetRPM=0;
    public static double hoodAngle=0;
    Pose2d robopose=new Pose2d();
    double target = 0;
    boolean shoot = false;


    @Override
    public void init() {
        Superstructure.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
    }

    @Override
    public void loop() {
        //robot.clearBulkCache();

        Superstructure.drivetrain.set(-gamepad1.left_stick_x,gamepad1.left_stick_y, -gamepad1.right_stick_x,new Rotation2d(Superstructure.drivetrain.OdometryModule.getHeading(AngleUnit.RADIANS)));

        if (gamepad1.right_bumper) {
            Superstructure.setCurrentWantedState(Superstructure.wantedState.SHOOT);
        }else if (gamepad1.cross) {
            Superstructure.setCurrentWantedState(Superstructure.wantedState.IDLE);
        }

        if(gamepad1.right_trigger>0.5){
            Superstructure.intake.setIntakeState(Intake.Systemstate.INTAKE);
        } else if (gamepad1.left_trigger>0.5) {
            Superstructure.intake.setIntakeState(Intake.Systemstate.EXHAUST);
        }else{
            Superstructure.intake.setIntakeState(Intake.Systemstate.IDLE);

        }


        if(Superstructure.vision.robotPose!=null){
            robopose =Superstructure.vision.robotPose;
        }

//        telemetry.addData("TargetRPM",Superstructure.flywheel.getShooterRPM());
//        telemetry.addData("hood angle",Superstructure.hood.getHoodAngle());
//        telemetry.addData("revolver angle",Superstructure.revolver.getRevolverAngle().getDegrees());
//        telemetry.addData("voltage",Superstructure.voltage);
//        telemetry.addData("shotter sensor",Superstructure.revolver.shooterSensor.getDistance(DistanceUnit.MM));
//        telemetry.addData("ready",Superstructure.revolver.sensorIndex);
//        telemetry.addData("ty",Superstructure.vision.ty);
//        telemetry.addData("color",Superstructure.revolver.currentRightColor);
//
//        telemetry.addData("VisionX",robopose.getX());
//        telemetry.addData("VisionY",robopose.getY());
//        telemetry.addData("Heading",robopose.getRotation().getDegrees());
        telemetry.addData("red",Superstructure.revolver.rightSensor.getNormalizedColors().red);
        telemetry.addData("green",Superstructure.revolver.rightSensor.getNormalizedColors().green);
        telemetry.addData("blue",Superstructure.revolver.rightSensor.getNormalizedColors().blue);


        Superstructure.read();
        Superstructure.periodic();
        Superstructure.write();


    }
}
