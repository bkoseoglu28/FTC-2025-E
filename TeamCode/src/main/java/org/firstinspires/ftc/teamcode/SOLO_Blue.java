package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure.follower;
import static org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure.revolver;
import static org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure.setVoltage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.lib.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.lib.joysticklib.Toggle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

@Config
@TeleOp(name = "SOLO_Blue")
public class SOLO_Blue extends OpMode {
    static FtcDashboard dashboard;
//    public static double targetRPM=0;
//    public static double hoodAngle=0;
    Pose2d robopose = new Pose2d();
    double target = 0;
    boolean shoot = false;
    Toggle vibrator;
    //MecanumDrivetrain drivetrain=new MecanumDrivetrain();

    @Override
    public void init() {
        Superstructure.setIsBlue(true);
        Superstructure.init(hardwareMap);
        Superstructure.reset();
        //drivetrain.init(hardwareMap);
        vibrator=new Toggle(false);
        follower.update();

//        dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        //robot.clearBulkCache();
        setVoltage(()-> hardwareMap.voltageSensor.iterator().next().getVoltage());

        //drivetrain.set(-gamepad1.left_stick_x,gamepad1.left_stick_y, 0.7*-gamepad1.right_stick_x, new Rotation2d(drivetrain.OdometryModule.getHeading(AngleUnit.RADIANS)));
        follower.setTeleOpDrive(-gamepad1.left_stick_x,gamepad1.left_stick_y, 0.5*-gamepad1.right_stick_x);
        if (gamepad1.options) {
//            drivetrain.OdometryModule.resetPosAndIMU();
//            drivetrain.OdometryModule.recalibrateIMU();
//            localizer.setPose(globalTagPosition);
        }

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
        //Superstructure.setAngularVel(Units.degreesToRadians(drivetrain.OdometryModule.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)));

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
        //telemetry.addData("heading", drivetrain.OdometryModule.getPose2d().getRotation().getDegrees());
//        telemetry.addData("revolver angle",Superstructure.revolver.getRevolverAngle().getDegrees());
        telemetry.addData("revolver target pose",Superstructure.revolver.RevolverController.getTargetPosition());
//        telemetry.addData("revolver ıs",Superstructure.revolver.IsAtSetpoint());
        telemetry.addData("ty",Superstructure.vision.ty);
        telemetry.addData("tv",Superstructure.vision.tv);

        telemetry.addData("shooter Sensor", revolver.shooterSensor.getDistance(DistanceUnit.MM));

//        sendPoseToDashboard(robopose);

        Superstructure.read();
        //drivetrain.read();
        Superstructure.periodic();
        //drivetrain.periodic();
        Superstructure.write();
        //drivetrain.write();


    }}
//    public static void sendPoseToDashboard(Pose2d pose) {
//        TelemetryPacket packet = new TelemetryPacket();
//
//        // Add pose data as telemetry fields
//        packet.put("x", DistanceUnit.INCH.fromMeters(pose.getX()));
//        packet.put("y", DistanceUnit.INCH.fromMeters(pose.getY()));
//        packet.put("heading (deg)", Math.toDegrees(pose.getRotation().getRadians()));
//
//        // Optionally draw the robot on the field overlay
//        packet.fieldOverlay()
//                .setStroke("blue")
//                .strokeCircle(DistanceUnit.INCH.fromMeters(pose.getX()), DistanceUnit.INCH.fromMeters(pose.getY()), 5)
//                .setStroke("red")
//                .strokeLine(
//                        DistanceUnit.INCH.fromMeters(pose.getX()),
//                        DistanceUnit.INCH.fromMeters(pose.getY()),
//                        DistanceUnit.INCH.fromMeters(pose.getX()) + 10 * Math.cos(pose.getRotation().getRadians()),
//                        DistanceUnit.INCH.fromMeters(pose.getY()) + 10 * Math.sin(pose.getRotation().getRadians())
//                );
//
//        dashboard.sendTelemetryPacket(packet);
//    }
//}

