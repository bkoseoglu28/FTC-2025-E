package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.MathUtils;
import org.firstinspires.ftc.teamcode.lib.RTPAxon;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.Sensors;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Feeder.Feeder;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
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
    double turretsetpoint=0;
    Pose2d robopose=new Pose2d();


    @Override
    public void init() {
        Superstructure.init(hardwareMap);
        turretsetpoint=0;
    }

    @Override
    public void loop() {
        //robot.clearBulkCache();

        Superstructure.drivetrain.set(-gamepad1.left_stick_x,gamepad1.left_stick_y, -gamepad1.right_stick_x,new Rotation2d(Superstructure.drivetrain.OdometryModule.getHeading(AngleUnit.RADIANS)));

        double anglesetpoint=new Rotation2d(Units.degreesToRadians(Superstructure.turret.getTurretAngle())).minus(new Rotation2d(Units.degreesToRadians(Superstructure.vision.tx))).getDegrees();
        Superstructure.turret.setTurretAngle(anglesetpoint);

//        if(gamepad1.dpad_up){
//            robot.axonController.changeTargetRotation(1);
//        } else if (gamepad1.dpad_down) {
//            robot.axonController.changeTargetRotation(-1);
//        }


        if(gamepad1.triangle){
            Superstructure.flywheel.setSetpointRPM(4500);
            //robot.TurretController.setTargetPosition(135);
            //RevolverController.setTargetPosition(120);
            //robot.RevolverController.setTargetPosition(120);
        }else if(gamepad1.square){
            Superstructure.flywheel.setSetpointRPM(2250);
            //robot.TurretController.setTargetPosition(-135);
            //RevolverController.setTargetPosition(-120);
            //robot.RevolverController.setTargetPosition(-120);
        }
        else if(gamepad1.cross){
            Superstructure.flywheel.setSetpointRPM(0);
            //robot.TurretController.setTargetPosition(0);
            //RevolverController.setTargetPosition(0);
            //robot.RevolverController.setTargetPosition(0);
        }

        if (gamepad1.right_bumper) {
            //robot.axonController.setTargetRotation(24);
            if((Superstructure.flywheel.getShooterRPM()>4000)){
                //robot.LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                Superstructure.revolver.setRevolverAngle(120);
                Superstructure.feeder.setFeederState(Feeder.Systemstate.FEED);
            }else{
                //robot.LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
            }
        }else{
            //robot.axonController.setTargetRotation(1);
            Superstructure.feeder.setFeederState(Feeder.Systemstate.IDLE);
            Superstructure.revolver.setRevolverAngle(0);
            //robot.LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        }

        if(Superstructure.vision.robotPose!=null){
            robopose =Superstructure.vision.robotPose;
        }


//        telemetry.addData("Measured RPM",VelocityController.getVelocity());
//        telemetry.addData("Shooter Velocity",upShooterMotor.getVelocity());
//        telemetry.addData("Turret pose",robot.doubleSubscriber(Sensors.SensorType.TURRETENCODER));
//        telemetry.addData("Revolver pose",robot.doubleSubscriber(Sensors.SensorType.REVOLVERENCODER));
//        telemetry.addData("Flywheel vel",robot.VelocityController.getVelocity());
//        telemetry.addData("x",robot.OdometryModule.getPose2d().getX());
//        telemetry.addData("tx", robot.LL3.getLatestResult().getTx());
//        telemetry.addData("y",robot.OdometryModule.getPose2d().getY());
//        telemetry.addData("r",robot.OdometryModule.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("obelisk",robot.vision.currentObelisk);
        telemetry.addData("TargetRPM",Superstructure.flywheel.getShooterRPM());
        telemetry.addData("hood angle",Superstructure.hood.getHoodAngle());
        telemetry.addData("VisionX",robopose.getX());
        telemetry.addData("VisionY",robopose.getY());
        telemetry.addData("Heading",robopose.getRotation().getDegrees());

        Superstructure.read();
        Superstructure.periodic();
        Superstructure.write();


    }
}
