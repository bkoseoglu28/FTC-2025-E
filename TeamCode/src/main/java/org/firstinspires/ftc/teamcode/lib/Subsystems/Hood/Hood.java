package org.firstinspires.ftc.teamcode.lib.Subsystems.Hood;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import edu.wpi.first.util.Util;
@Config
public class Hood extends WSubsystem {

//    public static double kp=0.002;
//    public static double ki=0.005;
//    public static double kd=0.0;
    CRServoImplEx HoodMotor;
    AnalogInput HoodEncoder;
    org.firstinspires.ftc.teamcode.lib.RTPAxon axonController;
    @Override
    public void init(HardwareMap hardwareMap) {
        HoodEncoder  = hardwareMap.get(AnalogInput.class, "hoodEncoder");
        HoodMotor = hardwareMap.get(CRServoImplEx.class, "HoodMotor");

        axonController = new org.firstinspires.ftc.teamcode.lib.RTPAxon(HoodMotor,HoodEncoder, org.firstinspires.ftc.teamcode.lib.RTPAxon.Direction.FORWARD);
        axonController.setPidCoeffs(0.002,0.005,0.0);
//        axonController.setPidCoeffs(0.0055,0,0.0000002);

    }
    public boolean IsAtSetpoint(){
        return Util.epsilonEquals(axonController.getTotalRotation(),axonController.getTargetRotation(),2);
    }
    double handleHoodSetpoint(double setpoint){
        if(setpoint<0.2){
            return 0.2;
        } else if (setpoint>32) {
            return 32;
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
//        axonController.setPidCoeffs(kp,ki,kd);
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
