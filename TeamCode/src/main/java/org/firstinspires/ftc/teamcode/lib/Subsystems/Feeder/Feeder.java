package org.firstinspires.ftc.teamcode.lib.Subsystems.Feeder;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

public class Feeder extends WSubsystem {

    CRServo lFeederMotor;
    CRServo rFeederMotor;
    CRServo blFeederMotor;
    CRServo brFeederMotor;
    Systemstate currentState;
    public enum Systemstate{
        IDLE,
        FEED
    }
    @Override
    public void init(HardwareMap hardwareMap) {
        lFeederMotor = hardwareMap.get(CRServo.class,"lFeederMotor");
        lFeederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rFeederMotor = hardwareMap.get(CRServo.class,"rFeederMotor");
        rFeederMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blFeederMotor = hardwareMap.get(CRServo.class, "blFeederMotor");
        blFeederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brFeederMotor = hardwareMap.get(CRServo.class, "brFeederMotor");
        brFeederMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    void setFeedersMotor(double speed){
        lFeederMotor.setPower(speed);
        rFeederMotor.setPower(speed);
        blFeederMotor.setPower(speed);
        brFeederMotor.setPower(speed);
    }
    public void setFeederState(Systemstate st){
        if(currentState!=st){
            this.currentState=st;
        }
    }
    @Override
    public void periodic() {
        switch (currentState){
            case IDLE:
                setFeedersMotor(0);
                break;
            case FEED:
                setFeedersMotor(1);
                break;
            default:
                setFeederState(Systemstate.IDLE);
                break;
        }
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {

    }
}
