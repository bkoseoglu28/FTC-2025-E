package org.firstinspires.ftc.teamcode.lib.Subsystems.Revolver;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;

public class Revolver extends WSubsystem {
    DcMotorEx revolverMotor;
    WEncoder RevolverEncoder;
    public WActuatorGroup RevolverController;
    public RevColorSensorV3 shooterSensor;
    public RevColorSensorV3 feederSensor;
    public RevColorSensorV3 rightSensor;
    public RevColorSensorV3 leftSensor;
    public colors currentRightColor= colors.UNKNOWN;
    public colors currentLeftColor= colors.UNKNOWN;
    public colors currentFeeederColor= colors.UNKNOWN;

    public Slot currentSlot;
    public boolean startVibrating=false;
    public double startedTime;
    public boolean manualadjust=false;

    public int sensorIndex=0;
    public enum colors{
        PURPLE,
        GREEN,
        UNKNOWN
    }
    public Slot slot1=new Slot(-120);
    public Slot slot2=new Slot(0);
    public Slot slot3=new Slot(120);

    @Override
    public void init(HardwareMap hardwareMap) {
        revolverMotor = hardwareMap.get(DcMotorEx.class,"revolverMotor");
        revolverMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        revolverMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RevolverEncoder = new WEncoder(new MotorEx(hardwareMap,"revolverMotor").encoder);
        RevolverController = new WActuatorGroup(()-> getRevolverAngle().getDegrees(),revolverMotor)
                .setPIDController(new edu.wpi.first.math.controller.PIDController(0.05,0,0))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,0.1)
                .enableContinuousInput(180,-180);
        RevolverController.setMaxPower(0.375);
        RevolverEncoder.encoder.reset();
        shooterSensor=hardwareMap.get(RevColorSensorV3.class,"shooterSensor");
        leftSensor= hardwareMap.get(RevColorSensorV3.class,"leftSensor");
        leftSensor.setGain(10);
        rightSensor= hardwareMap.get(RevColorSensorV3.class,"rightSensor");
        rightSensor.setGain(10);
        feederSensor= hardwareMap.get(RevColorSensorV3.class,"feederSensor");
        feederSensor.setGain(10);
        currentSlot=slot2;
    }
    public boolean IsAtSetpoint(){
        RevolverController.getController().setTolerance(25);
        return RevolverController.getController().atSetpoint();
    }
    double vibrationHz = 0.0;
    double vibrationAmplitude = 0.0;
    public void stopVibration() {
        startVibrating = false;
        vibrationHz = 0.0;
        vibrationAmplitude = 0.0;
        startedTime = 0.0;
    }
    public void setVibrating(boolean st){
        if(startVibrating!=st){
            startVibrating=st;
        }
    }
    public void setManualAdjust(boolean ma){
        if(manualadjust!=ma){
            manualadjust=ma;
        }
    }

    public boolean isEmpty(){
        return !slot1.IsthereBall()&&!slot2.IsthereBall()&&!slot3.IsthereBall();
    }
    
    public Rotation2d getRevolverAngle(){
        double encoderRots= RevolverEncoder.getPosition();

        return new Rotation2d(encoderRots*(1.0/((((1+(46.0/17))) * (1+(46/17.0))) * (1+(46.0/17)) * 28))*(16.0/30.0)*2*Math.PI);
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
    public void setVibration(double hz,double angle){
        vibrationHz = hz;
        vibrationAmplitude = angle;
        if(startedTime==0){
            startedTime=System.nanoTime() / 1e9;
        }
        double t = (System.nanoTime() / 1e9) - startedTime; // seconds since start
        setRevolverAngle(getRevolverAngle().getDegrees() + angle * Math.sin(2 * Math.PI * hz * t));
    }

    public void setVoltage(DoubleSupplier voltage){
        RevolverController.setVoltageSupplier(voltage);
    }
    public void getLeftColor(){
        double normred =leftSensor.getNormalizedColors().red;
        double normblue =leftSensor.getNormalizedColors().blue;
        double normgreen =leftSensor.getNormalizedColors().green;

        if(normblue > 0.013 && normgreen > 0.018 && normred < 0.01){
            currentLeftColor = colors.GREEN;
        } else if(normblue > 0.018 && normgreen > 0.013 && normred > 0.01){
            currentLeftColor = colors.PURPLE;
        } else {
            currentLeftColor = colors.UNKNOWN;
        }
    }
    public void getRightColor(){
        double normred =rightSensor.getNormalizedColors().red;
        double normblue =rightSensor.getNormalizedColors().blue;
        double normgreen =rightSensor.getNormalizedColors().green;

        if(normblue > 0.013 && normgreen > 0.018 && normred < 0.01){
            currentRightColor = colors.GREEN;
        } else if(normblue > 0.015 && normgreen > 0.01 && normred > 0.0085){
            currentRightColor = colors.PURPLE;
        } else {
            currentRightColor = colors.UNKNOWN;
        }
    }
    public void getFeederColor(){
        double normred =feederSensor.getNormalizedColors().red;
        double normblue =feederSensor.getNormalizedColors().blue;
        double normgreen =feederSensor.getNormalizedColors().green;

        if(normblue < 0.18 && normgreen > 0.09 && normred < 0.04){
            currentFeeederColor = colors.GREEN;
        } else if(normblue < 0.22 && normgreen < 0.13 && normred > 0.04){
            currentFeeederColor = colors.PURPLE;
        } else {
            currentFeeederColor = colors.UNKNOWN;
        }
    }
    public void setSlots(double currentAngle){
        if(IsAtSetpoint()){
            if(currentAngle== slot1.getAngle()){
                slot1.setColor(currentFeeederColor);
                slot2.setColor(currentRightColor);
                slot3.setColor(currentLeftColor);
                slot2.setIsthereBall(rightSensor.getDistance(DistanceUnit.MM)<55);
                slot3.setIsthereBall(leftSensor.getDistance(DistanceUnit.MM)<55);
                slot1.setIsthereBall(feederSensor.getDistance(DistanceUnit.MM)<50);
                currentSlot=slot1;
            } else if (currentAngle== slot2.getAngle()) {
                slot2.setColor(currentFeeederColor);
                slot3.setColor(currentRightColor);
                slot1.setColor(currentLeftColor);
                slot3.setIsthereBall(rightSensor.getDistance(DistanceUnit.MM)<55);
                slot1.setIsthereBall(leftSensor.getDistance(DistanceUnit.MM)<55);
                slot2.setIsthereBall(feederSensor.getDistance(DistanceUnit.MM)<50);
                currentSlot=slot2;
            }else if (currentAngle== slot3.getAngle()) {
                slot3.setColor(currentFeeederColor);
                slot2.setColor(currentLeftColor);
                slot1.setColor(currentRightColor);
                slot2.setIsthereBall(leftSensor.getDistance(DistanceUnit.MM)<55);
                slot1.setIsthereBall(rightSensor.getDistance(DistanceUnit.MM)<55);
                slot3.setIsthereBall(feederSensor.getDistance(DistanceUnit.MM)<50);

                currentSlot=slot3;
            }
        }

    }

    public void setRevolverAngle(double angle){
        RevolverController.setTargetPosition(angle);
    }
    @Override
    public void periodic() {
        RevolverController.periodic();
    }

    @Override
    public void read() {

        setSlots(RevolverController.getTargetPosition());
        getLeftColor();
        getFeederColor();
        getRightColor();
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
