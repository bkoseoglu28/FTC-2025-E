package org.firstinspires.ftc.teamcode.lib.Subsystems.Hood;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

public class Hood extends WSubsystem {

    CRServoImplEx HoodMotor;
    AnalogInput HoodEncoder;
    org.firstinspires.ftc.teamcode.lib.RTPAxon axonController;
    @Override
    public void init(HardwareMap hardwareMap) {
        HoodEncoder  = hardwareMap.get(AnalogInput.class, "hoodEncoder");
        HoodMotor = hardwareMap.get(CRServoImplEx.class, "HoodMotor");

        axonController = new org.firstinspires.ftc.teamcode.lib.RTPAxon(HoodMotor,HoodEncoder, org.firstinspires.ftc.teamcode.lib.RTPAxon.Direction.FORWARD);
        axonController.setPidCoeffs(0.009,0,0);
    }
    public void setHoodAngle(double angle){
        axonController.setTargetRotation(angle);
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
