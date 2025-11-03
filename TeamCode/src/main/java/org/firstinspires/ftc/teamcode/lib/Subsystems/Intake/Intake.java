package org.firstinspires.ftc.teamcode.lib.Subsystems.Intake;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

public class Intake extends WSubsystem {

    CRServo masterIntakeMotor;
    CRServo slaveIntakeMotor;

    CRServo downIntakeMotor;
    Systemstate currentState= Systemstate.IDLE;
    public enum Systemstate{
        IDLE,
        INTAKE,
        EXHAUST
    }
    @Override
    public void init(HardwareMap hardwareMap) {
        masterIntakeMotor = hardwareMap.get(CRServo.class,"masterIntakeMotor");
        masterIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slaveIntakeMotor = hardwareMap.get(CRServo.class,"slaveIntakeMotor");
        slaveIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        downIntakeMotor = hardwareMap.get(CRServo.class,"downIntakeMotor");
        downIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    void setIntakeMotor(double speed){
        masterIntakeMotor.setPower(speed);
        slaveIntakeMotor.setPower(speed);
        downIntakeMotor.setPower(speed);
    }
    public void setIntakeState(Systemstate st){
        if(currentState!=st){
            this.currentState=st;
        }
    }
    @Override
    public void periodic() {
        switch (currentState){
            case IDLE:
                setIntakeMotor(0);
                break;
            case INTAKE:
                setIntakeMotor(1);
                break;
            case EXHAUST:
                setIntakeMotor(-1);
                break;
            default:
                setIntakeState(Systemstate.IDLE);
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
