package org.firstinspires.ftc.teamcode.lib.Subsystems.Revolver;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.Sensors;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.Util;

public class Revolver extends WSubsystem {

    DcMotorEx revolverMotor;
    WEncoder RevolverEncoder;
    WActuatorGroup RevolverController;
    public RevColorSensorV3 shooterSensor;
    public int sensorIndex=0;

    @Override
    public void init(HardwareMap hardwareMap) {
        revolverMotor = hardwareMap.get(DcMotorEx.class,"revolverMotor");
        revolverMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        revolverMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RevolverEncoder = new WEncoder(new MotorEx(hardwareMap,"revolverMotor").encoder);
        RevolverController = new WActuatorGroup(()-> getRevolverAngle().getDegrees(),revolverMotor)
                .setPIDController(new edu.wpi.first.math.controller.PIDController(0.017,0.0085,0.0004))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,0.16)
                .setErrorTolerance(0.1)
                .enableContinuousInput(180,-180);
        RevolverController.setMaxPower(0.5);
        RevolverEncoder.encoder.reset();
        shooterSensor=hardwareMap.get(RevColorSensorV3.class,"shooterSensor");
    }
    public boolean IsAtSetpoint(){
        return Util.epsilonEquals(getRevolverAngle().getDegrees(),RevolverController.getTargetPosition(),10);
    }
    public Rotation2d getRevolverAngle(){
        double encoderRots= RevolverEncoder.getPosition()/28;

        return new Rotation2d(encoderRots*(1.0/(((1+(46.0/17.0))) * (1+(46.0/17.0))))*(16.0/30.0)*2*Math.PI);
    }
    public void handleShooterSensor(){
        if(shooterSensor.getDistance(DistanceUnit.MM)>30){
            sensorIndex=1;
        } else if (sensorIndex==1&&shooterSensor.getDistance(DistanceUnit.MM)<50) {
            sensorIndex=2;
        } else if (!IsAtSetpoint()||Superstructure.currentSystemState== Superstructure.systemState.IDLE) {
            sensorIndex=0;
        }

    }
    public void setRevolverAngle(double angle){
        RevolverController.setTargetPosition(angle);
    }
    @Override
    public void periodic() {
        RevolverController.setVoltageSupplier(Superstructure.voltage);
        RevolverController.periodic();
    }

    @Override
    public void read() {
        handleShooterSensor();
        RevolverController.read();
    }

    @Override
    public void write() {
        RevolverController.write();
    }

    @Override
    public void reset() {

    }
}
