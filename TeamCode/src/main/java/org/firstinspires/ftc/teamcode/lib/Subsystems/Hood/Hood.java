package org.firstinspires.ftc.teamcode.lib.Subsystems.Hood;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import edu.wpi.first.util.Util;

public class Hood extends WSubsystem {
    CRServoImplEx HoodMotor;
    AnalogInput HoodEncoder;
    org.firstinspires.ftc.teamcode.lib.RTPAxon axonController;
    @Override
    public void init(HardwareMap hardwareMap) {
        HoodEncoder  = hardwareMap.get(AnalogInput.class, "hoodEncoder");
        HoodMotor = hardwareMap.get(CRServoImplEx.class, "HoodMotor");

        axonController = new org.firstinspires.ftc.teamcode.lib.RTPAxon(HoodMotor,HoodEncoder, org.firstinspires.ftc.teamcode.lib.RTPAxon.Direction.FORWARD);
        axonController.setPidCoeffs(0.0065,0,0.000005);
    }
    public boolean IsAtSetpoint(){
        return Util.epsilonEquals(axonController.getTotalRotation(),axonController.getTargetRotation(),2);
    }
    double handleHoodSetpoint(double setpoint){
        if(setpoint<0.2){
            return 0.2;
        } else if (setpoint>33) {
            return 33;
        }
        return setpoint;
    }
    public void setHoodAngle(double angle){
        axonController.setTargetRotation(handleHoodSetpoint(angle));
    }
    public double getHoodAngle(){
        return axonController.getTotalRotation();
    }

    @Override
    public void periodic() {
    axonController.update();
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
