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
import org.firstinspires.ftc.teamcode.lib.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Revolver.Revolver;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Vision.FieldAprilTags;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.lib.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.lib.math.InterpolatingDouble;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

@Config
public class Superstructure {
    public static double setpointRPM=0;
    public static double setpointHood=0;
    static HardwareMap hardwaremap;
    public static Feeder feeder=new Feeder();
    public static Flywheel flywheel=new Flywheel();
    public static Hood hood=new Hood();
    public static Revolver revolver=new Revolver();
    public static Turret turret=new Turret();
    public static MecanumDrivetrain drivetrain=new MecanumDrivetrain();
    public static double voltage=0;
    public static Vision vision=new Vision();
    public static Intake intake = new Intake();

    public static PIDController visionTargeting;
    public static systemState currentSystemState = systemState.IDLE;
    public static wantedState currentWatedState = wantedState.IDLE;
    public static int revolverTarget=-120;
    static RevBlinkinLedDriver LEDS;
    public static boolean ready;
    public static double lastknownangle = -37;
    static double filteredTx=0;
    static double mT=0;
    static double hint=0;
    static double turretHintAdjustment=30;



    public static enum systemState{
        IDLE,
        AIMING,
        SHOOTING,
        SHOOTING_OBELISK,
        INTAKING
    }
    public static enum wantedState{
        SHOOT,
        INTAKE,
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
        intake.init(hardwareMap);
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
            case INTAKE:
                setCurrentSystemState(systemState.INTAKING);
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

        double velcompdegrees = -0.213*Units.radiansToDegrees(robotangularvel);
        filteredTx = 0.2 * vision.tx + (1 - 0.2) * filteredTx;
        double compensatetx = -filteredTx+ velcompdegrees;
        double error;
        if(vision.tv){
            error= turret.getTurretAngle()+compensatetx;
            error= new Rotation2d(Units.degreesToRadians(error)).getDegrees()+ (Timer.getFPGATimestamp() -mT) * -Units.radiansToDegrees(robotangularvel);
            mT=Timer.getFPGATimestamp();
            turret.setTurretAngle(error);
            lastknownangle=error+drivetrain.OdometryModule.getHeading(AngleUnit.DEGREES);
        }else{
            if(turret.getTurretAngle()>=120){
                hint=-30;
            }else if(turret.getTurretAngle()<=-120){
                hint=30;
            }
            turretHintAdjustment=turret.getTurretAngle();
            turretHintAdjustment+=hint;
            turret.setTurretAngle(turretHintAdjustment);
        }

        hood.setHoodAngle(Constants.ShootingParams.kHoodMap.getInterpolated(new InterpolatingDouble(Superstructure.vision.ty)).value);

        switch (currentSystemState){
            case IDLE:
                if(revolver.startVibrating){
                    revolver.setVibration(5,45);
                }else{
                    if(revolver.currentSlot.IsthereBall()){
                        revolverTarget= (int) revolver.currentSlot.getAngle();
                    }else{
                        if(revolver.currentSlot==revolver.slot1){
                            if(revolver.slot2.IsthereBall()){
                                revolverTarget= (int) revolver.slot2.getAngle();
                            }else if (revolver.slot3.IsthereBall()){
                                revolverTarget= (int) revolver.slot3.getAngle();
                            }
                        } else if (revolver.currentSlot==revolver.slot2) {
                            if(revolver.slot1.IsthereBall()){
                                revolverTarget= (int) revolver.slot1.getAngle();
                            }else if (revolver.slot3.IsthereBall()){
                                revolverTarget= (int) revolver.slot3.getAngle();
                            }
                        } else if (revolver.currentSlot==revolver.slot3) {
                            if(revolver.slot1.IsthereBall()){
                                revolverTarget= (int) revolver.slot1.getAngle();
                            }else if (revolver.slot2.IsthereBall()){
                                revolverTarget= (int) revolver.slot2.getAngle();
                            }
                        }
                    }
                    revolver.stopVibration();
                    revolver.startedTime=0;
                }
                //turret.setTurretAngle(0);
                flywheel.setSetpointRPM(1500);
                feeder.setFeederState(Feeder.Systemstate.IDLE);
                intake.setIntakeState(Intake.Systemstate.IDLE);
                LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                break;
            case AIMING:
                flywheel.setSetpointRPM(Constants.ShootingParams.kRPMMap.getInterpolated(new InterpolatingDouble(Superstructure.vision.ty)).value);
                feeder.setFeederState(Feeder.Systemstate.IDLE);
                intake.setIntakeState(Intake.Systemstate.IDLE);
                ready = flywheel.IsAtSetpoint()&& hood.IsAtSetpoint();
                    if(ready){
                        setCurrentSystemState(systemState.SHOOTING);
                    }
                    if(revolver.currentSlot.IsthereBall()){
                        revolverTarget= (int) revolver.currentSlot.getAngle();
                    }else{
                       if(revolver.currentSlot==revolver.slot1){
                           if(revolver.slot2.IsthereBall()){
                               revolverTarget= (int) revolver.slot2.getAngle();
                           }else if (revolver.slot3.IsthereBall()){
                               revolverTarget= (int) revolver.slot3.getAngle();
                           }
                       } else if (revolver.currentSlot==revolver.slot2) {
                           if(revolver.slot1.IsthereBall()){
                               revolverTarget= (int) revolver.slot1.getAngle();
                           }else if (revolver.slot3.IsthereBall()){
                               revolverTarget= (int) revolver.slot3.getAngle();
                           }
                       } else if (revolver.currentSlot==revolver.slot3) {
                           if(revolver.slot1.IsthereBall()){
                               revolverTarget= (int) revolver.slot1.getAngle();
                           }else if (revolver.slot2.IsthereBall()){
                               revolverTarget= (int) revolver.slot2.getAngle();
                           }
                       }
                    }
                revolver.setRevolverAngle(revolverTarget);
                LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
                break;
            case SHOOTING:
                    if(revolver.sensorIndex==2){
                        setCurrentSystemState(systemState.AIMING);
                    }

                revolver.setRevolverAngle(revolverTarget);
                LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                flywheel.setSetpointRPM(Constants.ShootingParams.kRPMMap.getInterpolated(new InterpolatingDouble(Superstructure.vision.ty)).value);
                feeder.setFeederState(Feeder.Systemstate.FEED);
                intake.setIntakeState(Intake.Systemstate.IDLE);
                break;
            case INTAKING:
//                if(revolver.IsAtSetpoint()){
//                if((revolver.slot1.IsthereBall()&&!revolver.slot2.IsthereBall()&&!revolver.slot3.IsthereBall())||
//                        (revolver.slot1.IsthereBall()&&revolver.slot2.IsthereBall()&&!revolver.slot3.IsthereBall())||
//                        (revolver.slot1.IsthereBall()&&!revolver.slot2.IsthereBall()&&revolver.slot3.IsthereBall())){
//                    revolverTarget= (int) revolver.slot1.getAngle();
//                }else if((!revolver.slot1.IsthereBall()&&revolver.slot2.IsthereBall()&&!revolver.slot3.IsthereBall())||
//                        (!revolver.slot1.IsthereBall()&&revolver.slot2.IsthereBall()&&revolver.slot3.IsthereBall())){
//                    revolverTarget= (int) revolver.slot2.getAngle();
//                }else if(!revolver.slot1.IsthereBall()&&!revolver.slot2.IsthereBall()&&revolver.slot3.IsthereBall()){
//                    revolverTarget= (int) revolver.slot3.getAngle();
//                }}
                if(revolver.currentSlot.IsthereBall()){
                    revolverTarget= (int) revolver.currentSlot.getAngle();
                }else{
                    if(revolver.currentSlot==revolver.slot1){
                        if(revolver.slot2.IsthereBall()){
                            revolverTarget= (int) revolver.slot2.getAngle();
                        }else if (revolver.slot3.IsthereBall()){
                            revolverTarget= (int) revolver.slot3.getAngle();
                        }else{
                            revolverTarget= (int) revolver.slot1.getAngle();
                        }
                    } else if (revolver.currentSlot==revolver.slot2) {
                        if(revolver.slot1.IsthereBall()){
                            revolverTarget= (int) revolver.slot1.getAngle();
                        }else if (revolver.slot3.IsthereBall()){
                            revolverTarget= (int) revolver.slot3.getAngle();
                        }else{
                            revolverTarget= (int) revolver.slot2.getAngle();
                        }
                    } else if (revolver.currentSlot==revolver.slot3) {
                        if(revolver.slot1.IsthereBall()){
                            revolverTarget= (int) revolver.slot1.getAngle();
                        }else if (revolver.slot2.IsthereBall()){
                            revolverTarget= (int) revolver.slot2.getAngle();
                        }else{
                            revolverTarget= (int) revolver.slot3.getAngle();
                        }
                    }}
                revolver.setRevolverAngle(revolverTarget);
                LEDS.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                flywheel.setSetpointRPM(1500);
                feeder.setFeederState(Feeder.Systemstate.IDLE);
                intake.setIntakeState(Intake.Systemstate.INTAKE);

                break;
        }
        feeder.periodic();
        flywheel.periodic();
        hood.periodic();
        revolver.periodic();
        turret.periodic();
        vision.periodic();
        drivetrain.periodic();
        intake.periodic();
    }


    public static void read() {
        feeder.read();
        flywheel.read();
        hood.read();
        revolver.read();
        turret.read();
        vision.read();
        drivetrain.read();
        intake.read();
    }

    public static void write() {
        feeder.write();
        flywheel.write();
        hood.write();
        revolver.write();
        turret.write();
        vision.write();
        drivetrain.write();
        intake.write();
    }

    public static void reset() {
        feeder.reset();
        flywheel.reset();
        hood.reset();
        revolver.reset();
        turret.reset();
        vision.reset();
        drivetrain.reset();
        intake.reset();
    }

    public static void setVoltage(DoubleSupplier voltage) {
        flywheel.setVoltage(voltage);
        revolver.setVoltage(voltage);
        turret.setVoltage(voltage);
    }
}
