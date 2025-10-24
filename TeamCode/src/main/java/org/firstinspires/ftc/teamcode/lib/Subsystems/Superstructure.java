package org.firstinspires.ftc.teamcode.lib.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.Subsystems.Feeder.Feeder;
import org.firstinspires.ftc.teamcode.lib.Subsystems.FlyWheel.Flywheel;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Hood.Hood;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Revolver.Revolver;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.lib.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

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

    public static void init(HardwareMap hardwareMap) {
        feeder.init(hardwareMap);
        flywheel.init(hardwareMap);
        hood.init(hardwareMap);
        revolver.init(hardwareMap);
        turret.init(hardwareMap);
        vision.init(hardwareMap);
        drivetrain.init(hardwareMap);
        hardwaremap=hardwareMap;
    }


    public static void periodic() {
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
