package org.firstinspires.ftc.teamcode.lib.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.lib.Constants;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Feeder.Feeder;
import org.firstinspires.ftc.teamcode.lib.Subsystems.FlyWheel.Flywheel;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Hood.Hood;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Revolver.Revolver;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Vision.FieldAprilTags;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.lib.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.lib.math.InterpolatingDouble;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Superstructure {

    static HardwareMap hardwaremap;
    public static Feeder feeder=new Feeder();
    public static Flywheel flywheel=new Flywheel();
    public static Hood hood=new Hood();
    public static Revolver revolver=new Revolver();
    public static Turret turret=new Turret();
    public static MecanumDrivetrain drivetrain=new MecanumDrivetrain();
    public static double voltage=12;
    public static Vision vision=new Vision();

    public static PIDController visionTargeting;
    public static systemState currentSystemState = systemState.IDLE;
    public static wantedState currentWatedState = wantedState.IDLE;
    public static int revolverTarget=-120;
    static RevBlinkinLedDriver LEDS;
    public static boolean ready;



    public static enum systemState{
        IDLE,
        AIMING,
        SHOOTING
    }
    public static enum wantedState{
        SHOOT,
        IDLE
    }

    public static void init(HardwareMap hardwareMap) {
        feeder.init(hardwareMap);
        flywheel.init(hardwareMap);
        hood.init(hardwareMap);
        revolver.init(hardwareMap);
        turret.init(hardwareMap);
        vision.init(hardwareMap);
        drivetrain.init(hardwareMap);
        hardwaremap=hardwareMap;
        LEDS = hardwareMap.get(RevBlinkinLedDriver.class, "ledDriver");

    }
    public static void setCurrentSystemState(systemState st){
        if(currentSystemState!=st)currentSystemState=st;
    }
    public static void setCurrentWantedState(wantedState st){
        if(currentWatedState!=st)currentWatedState=st;
    }


    public static void periodic() {
        switch (currentWatedState){
            case SHOOT:
                if(currentSystemState==systemState.IDLE){
                    setCurrentSystemState(systemState.AIMING);
                }
                break;
            case IDLE:
            default:
                setCurrentSystemState(systemState.IDLE);
                break;
        }
        Rotation2d fieldTurretAngle = new Rotation2d(Units.degreesToRadians(turret.getTurretAngle())).minus(new Rotation2d(Units.degreesToRadians(vision.tx)));

// Vision hedefini world-relative hale getir

// PID veya direkt düzeltme uygula
//        if(Superstructure.vision.tv){
//            turret.setTurretAngle(fieldTurretAngle.getDegrees());
//        }else{
//            turret.setTurretAngle(new Rotation2d(drivetrain.OdometryModule.getHeading(AngleUnit.RADIANS)).unaryMinus().minus(fieldTurretAngle).getDegrees());
//        }
//        Translation2d robotToTurret = new Translation2d(-65.0/1000, 0);
//        Translation2d turrettorobot=drivetrain.OdometryModule.getPose2d().getTranslation().plus(robotToTurret);
//        turret.setTurretAngle(FieldAprilTags.TAG_20.toPose2d().getTranslation().minus(turrettorobot).getAngle().getDegrees());
        double robotangularvel = drivetrain.OdometryModule.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        double velcompdegrees = -0.19*Units.radiansToDegrees(robotangularvel);

        double compensatetx = -vision.tx+ velcompdegrees;
        double lastknownangle = 37;
        double error;
        if(vision.tv){
            error= turret.getTurretAngle()+compensatetx;
            error= new Rotation2d(Units.degreesToRadians(error)).getDegrees();
            turret.setTurretAngle(error);
            lastknownangle= turret.getTurretAngle()+drivetrain.OdometryModule.getHeading(AngleUnit.DEGREES);
        }else{
            turret.setTurretAngle(lastknownangle-drivetrain.OdometryModule.getHeading(AngleUnit.DEGREES));
        }




        hood.setHoodAngle(Constants.ShootingParams.kHoodMap.getInterpolated(new InterpolatingDouble(Superstructure.vision.ty)).value);

        switch (currentSystemState){
            case IDLE:
                //turret.setTurretAngle(0);
                revolverTarget=-120;
                flywheel.setSetpointRPM(1500);
                revolver.setRevolverAngle(revolverTarget);
                feeder.setFeederState(Feeder.Systemstate.IDLE);
                LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                break;
            case AIMING:
                //turret.setTurretAngle(90);
                flywheel.setSetpointRPM(Constants.ShootingParams.kRPMMap.getInterpolated(new InterpolatingDouble(Superstructure.vision.ty)).value);
                feeder.setFeederState(Feeder.Systemstate.IDLE);
                ready = flywheel.IsAtSetpoint()&& hood.IsAtSetpoint();
                if(ready){
                    setCurrentSystemState(systemState.SHOOTING);
                }
                LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
                break;
            case SHOOTING:
                if(revolver.sensorIndex==2){
                    revolverTarget+=120;
                    revolver.setRevolverAngle(revolverTarget);
                    setCurrentSystemState(systemState.AIMING);
                }
                LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                flywheel.setSetpointRPM(Constants.ShootingParams.kRPMMap.getInterpolated(new InterpolatingDouble(Superstructure.vision.ty)).value);
                feeder.setFeederState(Feeder.Systemstate.FEED);
                break;
        }
        voltage=hardwaremap.voltageSensor.iterator().next().getVoltage();
        feeder.periodic();
        flywheel.periodic();
        hood.periodic();
        revolver.periodic();
        turret.periodic();
        vision.periodic();
        drivetrain.periodic();
    }


    public static void read() {
        feeder.read();
        flywheel.read();
        hood.read();
        revolver.read();
        turret.read();
        vision.read();
        drivetrain.read();
    }

    public static void write() {
        feeder.write();
        flywheel.write();
        hood.write();
        revolver.write();
        turret.write();
        vision.write();
        drivetrain.write();
    }

    public void reset() {
        feeder.reset();
        flywheel.reset();
        hood.reset();
        revolver.reset();
        turret.reset();
        vision.reset();
        drivetrain.reset();
    }
}
