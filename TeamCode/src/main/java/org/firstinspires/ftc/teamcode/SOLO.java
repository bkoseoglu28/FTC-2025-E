package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure.revolver;
import static org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure.setVoltage;
import static org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure.turret;

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
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.MathUtils;
import org.firstinspires.ftc.teamcode.lib.RTPAxon;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.Sensors;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Feeder.Feeder;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Vision.FieldAprilTags;
import org.firstinspires.ftc.teamcode.lib.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.lib.joysticklib.Toggle;
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
    static FtcDashboard dashboard;
//    public static double targetRPM=0;
//    public static double hoodAngle=0;
    Pose2d robopose = new Pose2d();
    double target = 0;
    boolean shoot = false;
    Toggle vibrator;
    MecanumDrivetrain drivetrain;


    @Override
    public void init() {
        Superstructure.init(hardwareMap);
        Superstructure.reset();
        drivetrain.init(hardwareMap);
        drivetrain.setNewPose(new Pose2d(0,0,Rotation2d.fromDegrees(90)));
        vibrator=new Toggle(false);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
    }

    @Override
    public void loop() {
        //robot.clearBulkCache();
        setVoltage(()-> hardwareMap.voltageSensor.iterator().next().getVoltage());

        drivetrain.set(-gamepad1.left_stick_x,gamepad1.left_stick_y, 0.8*-gamepad1.right_stick_x, new Rotation2d(drivetrain.OdometryModule.getHeading(AngleUnit.RADIANS)));

        if (gamepad1.right_bumper) {
            Superstructure.setCurrentWantedState(Superstructure.wantedState.SHOOT);
        }else if (gamepad1.cross||!(gamepad1.right_trigger>0.5)&& Superstructure.currentWatedState!= Superstructure.wantedState.SHOOT) {
            Superstructure.setCurrentWantedState(Superstructure.wantedState.IDLE);
        }else if (gamepad1.right_trigger>0.5) {
            Superstructure.setCurrentWantedState(Superstructure.wantedState.INTAKE);
        }
//        if (gamepad1.dpad_up) {
//            turret.setTurretAngle(0);
//        }else if (gamepad1.dpad_left) {
//            turret.setTurretAngle(-90);
//        }else if (gamepad1.dpad_right) {
//            turret.setTurretAngle(90);
//        }

        Superstructure.setPanic(gamepad1.triangle);

        if(Superstructure.vision.RobotPose!=null){
            robopose =Superstructure.vision.RobotPose;
        }

        vibrator.update(gamepad1.ps);
        revolver.setVibrating(vibrator.getState());
        revolver.setManualAdjust(gamepad1.touchpad);
//      telemetry.addData("TargetRPM",Superstructure.flywheel.getShooterRPM());
        telemetry.addData("hood angle",Superstructure.hood.getHoodAngle());
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
////        telemetry.addData("current slot",Superstructure.revolver.currentSlot);
//        telemetry.addData("slot1 ball",Superstructure.revolver.slot1.IsthereBall());
//        telemetry.addData("slot1 color",Superstructure.revolver.slot1.getColor());
//        telemetry.addData("slot2 ball",Superstructure.revolver.slot2.IsthereBall());
//        telemetry.addData("slot2 color",Superstructure.revolver.slot2.getColor());
//        telemetry.addData("slot3 ball",Superstructure.revolver.slot3.IsthereBall());
//        telemetry.addData("slot3 color",Superstructure.revolver.slot3.getColor());
//        telemetry.addData("feeder distance",Superstructure.revolver.feederSensor.getDistance(DistanceUnit.MM));
//        telemetry.addData("feeder red",Superstructure.revolver.feederSensor.getNormalizedColors().red);
//        telemetry.addData("feeder green",Superstructure.revolver.feederSensor.getNormalizedColors().green);
//        telemetry.addData("feeder blue",Superstructure.revolver.feederSensor.getNormalizedColors().blue);
//        telemetry.addData("feeder color",Superstructure.revolver.currentFeeederColor.name());
        telemetry.addData("TargetRPM",Superstructure.flywheel.getShooterRPM());

        telemetry.addData("turret Angle",Superstructure.turret.getTurretAngle());
        telemetry.addData("revolver angle",Superstructure.revolver.getRevolverAngle().getDegrees());
        telemetry.addData("heading", drivetrain.OdometryModule.getPose2d().getRotation().getDegrees());
//        telemetry.addData("revolver angle",Superstructure.revolver.getRevolverAngle().getDegrees());
        telemetry.addData("revolver target pose",Superstructure.revolver.RevolverController.getTargetPosition());
//        telemetry.addData("revolver ıs",Superstructure.revolver.IsAtSetpoint());
        telemetry.addData("ty",Superstructure.vision.ty);
        telemetry.addData("shooter Sensor", revolver.shooterSensor.getDistance(DistanceUnit.MM));

        sendPoseToDashboard(robopose);

        Superstructure.read();
        Superstructure.periodic();
        Superstructure.write();


    }
    public static void sendPoseToDashboard(Pose2d pose) {
        TelemetryPacket packet = new TelemetryPacket();

        // Add pose data as telemetry fields
        packet.put("x", DistanceUnit.INCH.fromMeters(pose.getX()));
        packet.put("y", DistanceUnit.INCH.fromMeters(pose.getY()));
        packet.put("heading (deg)", Math.toDegrees(pose.getRotation().getRadians()));

        // Optionally draw the robot on the field overlay
        packet.fieldOverlay()
                .setStroke("blue")
                .strokeCircle(DistanceUnit.INCH.fromMeters(pose.getX()), DistanceUnit.INCH.fromMeters(pose.getY()), 5)
                .setStroke("red")
                .strokeLine(
                        DistanceUnit.INCH.fromMeters(pose.getX()),
                        DistanceUnit.INCH.fromMeters(pose.getY()),
                        DistanceUnit.INCH.fromMeters(pose.getX()) + 10 * Math.cos(pose.getRotation().getRadians()),
                        DistanceUnit.INCH.fromMeters(pose.getY()) + 10 * Math.sin(pose.getRotation().getRadians())
                );

        dashboard.sendTelemetryPacket(packet);
    }
}

