package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.lib.MathUtils;
import org.firstinspires.ftc.teamcode.lib.RTPAxon;
import org.firstinspires.ftc.teamcode.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.wrappers.WVelocityGroup;

@Config
@TeleOp(name = "SOLO")
public class SOLO extends OpMode {
    public static double KP=0;
    public static double KI=0;
    public static double KD=0;
    CRServo lFeederMotor;
    CRServo rFeederMotor;
    CRServo blFeederMotor;
    CRServo brFeederMotor;
    DcMotorEx revolverMotor;
    CRServoImplEx HoodMotor;
    AnalogInput HoodEncoder;
    RTPAxon axonController;
    DcMotorEx upShooterMotor;
    DcMotorEx downShooterMotor;
    DcMotorEx turretMotor;
    int tickPerRev = 28;
    double targetRPM = 0.0;
    double shooterGearRatio = 4.0 / 3.0;

    WVelocityGroup VelocityController;
    WActuatorGroup TurretController;
    WActuatorGroup RevolverController;
    WEncoder FlywheelEncoder;
    WEncoder TurretEncoder;
    WEncoder RevolverEncoder;


    public double getShooterRPM(){
        double measuredTicksPerSec = upShooterMotor.getVelocity();
        double measuredMotorRpm = (measuredTicksPerSec / tickPerRev)* 60.0;
        return (measuredMotorRpm * shooterGearRatio);
    }
    public Rotation2d getTurretAngle(){
        double encoderRots= TurretEncoder.getPosition()/28;
        double RottoMech = encoderRots*(1.0/(1+(46.0/11.0)))*(16.0/112.0)*2*Math.PI;

        return new Rotation2d(RottoMech);
    }
    public Rotation2d getRevolverAngle(){
        double encoderRots= RevolverEncoder.getPosition()/28;
        double RottoMech = encoderRots*(1.0/(((1+(46.0/17.0))) * (1+(46.0/17.0))))*(16.0/30.0)*2*Math.PI;

        return new Rotation2d(RottoMech);
    }
    @Override
    public void init() {
        lFeederMotor = hardwareMap.get(CRServo.class,"lFeederMotor");
        lFeederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rFeederMotor = hardwareMap.get(CRServo.class,"rFeederMotor");
        rFeederMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blFeederMotor = hardwareMap.get(CRServo.class, "blFeederMotor");
        blFeederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brFeederMotor = hardwareMap.get(CRServo.class, "brFeederMotor");
        brFeederMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        revolverMotor = hardwareMap.get(DcMotorEx.class,"revolverMotor");
        revolverMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        revolverMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor = hardwareMap.get(DcMotorEx.class,"turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        HoodEncoder  = hardwareMap.get(AnalogInput.class, "hoodEncoder");
        HoodMotor = hardwareMap.get(CRServoImplEx.class, "HoodMotor");

        axonController = new RTPAxon(HoodMotor,HoodEncoder, RTPAxon.Direction.FORWARD);
        axonController.setPidCoeffs(0.009,0,0);

        upShooterMotor = hardwareMap.get(DcMotorEx.class,"upShooterMotor");
        downShooterMotor = hardwareMap.get(DcMotorEx.class,"downShooterMotor");
        upShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        downShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        upShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        downShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FlywheelEncoder = new WEncoder(new MotorEx(hardwareMap, "upShooterMotor").encoder);
        TurretEncoder = new WEncoder(new MotorEx(hardwareMap, "turretMotor").encoder);
        RevolverEncoder = new WEncoder(new MotorEx(hardwareMap,"revolverMotor").encoder);

        VelocityController = new WVelocityGroup(()-> FlywheelEncoder.getRawVelocity(),upShooterMotor,downShooterMotor)
                .setPIDController(new PIDController(0,0,0))
                .setFeedforwardSimple(1.5,0.0026,0.0001);
        TurretController = new WActuatorGroup(()->getTurretAngle().getDegrees(),turretMotor)
                .setPIDController(new PIDController(0.015,0.09,0.001))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,0);
        RevolverController = new WActuatorGroup(()->getRevolverAngle().getDegrees(),revolverMotor)
                .setPIDController(new PIDController(0.1,0,0.001))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,0);
        TurretEncoder.encoder.reset();
        RevolverEncoder.encoder.reset();

        RevolverController.setMaxPower(0.5);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
    }

    public void setFeedersMotor(double speed){
        lFeederMotor.setPower(speed);
        rFeederMotor.setPower(speed);
        blFeederMotor.setPower(speed);
        brFeederMotor.setPower(speed);
    }

    public void setRevolverMotor(double speed){

        revolverMotor.setPower(speed);
    }

    public void setShooterMotor(double TargetRpm){
        this.targetRPM = TargetRpm;
        double targetMotorRPM = TargetRpm / shooterGearRatio;
        double ticksPerSec = (targetMotorRPM/60.0) * tickPerRev;
        downShooterMotor.setVelocity(ticksPerSec);
        upShooterMotor.setVelocity(ticksPerSec);
    }

    public void setShooterSpeedMotor(double speed){
        upShooterMotor.setPower(speed);
        downShooterMotor.setPower(speed);
    }



    @Override
    public void loop() {
        //TurretController.setPID(KP,KI,KD);
        VelocityController.read();
        VelocityController.periodicImpl();
        VelocityController.write();
        TurretController.read();
        TurretController.periodic();
        TurretController.write();
        RevolverController.read();
        RevolverController.periodic();
        //RevolverController.write();
        double voltage=hardwareMap.voltageSensor.iterator().next().getVoltage();

        VelocityController.setVoltageSupplier(voltage);
        TurretController.setVoltageSupplier(()->voltage);
        RevolverController.setVoltageSupplier(()->voltage);



        axonController.update();
        if(gamepad1.dpad_up){
            axonController.changeTargetRotation(0.5);
        }else if(gamepad1.dpad_down){
            axonController.changeTargetRotation(-0.5);
        }

        if(gamepad1.triangle){
            VelocityController.setTargetVelocity(4500);
            TurretController.setTargetPosition(135);
            //RevolverController.setTargetPosition(120);
        }else if(gamepad1.square){
            VelocityController.setTargetVelocity(2250);
            TurretController.setTargetPosition(-135);
            //RevolverController.setTargetPosition(-120);
        }
        else if(gamepad1.cross){
            VelocityController.setTargetVelocity(0);
            TurretController.setTargetPosition(0);
            //RevolverController.setTargetPosition(0);
        }else{
            setFeedersMotor(0);
            setRevolverMotor(0);
        }

//        telemetry.addData("Target RPM",VelocityController.getTargetVelocity());
//        telemetry.addData("Measured RPM",VelocityController.getVelocity());
//        telemetry.addData("Shooter Velocity",upShooterMotor.getVelocity());
        telemetry.addData("Turret pose",getTurretAngle().getDegrees());
        telemetry.addData("Revolver pose",getRevolverAngle().getDegrees());
    }
}
