package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.lib.RTPAxon;
import org.firstinspires.ftc.teamcode.wrappers.WVelocityGroup;

@TeleOp(name = "example")
public class example extends OpMode {

    public CRServoImplEx HoodMotor;
    public AnalogInput HoodEncoder;
    public RTPAxon axonController;

    public WVelocityGroup VelocityController;
    @Override
    public void init() {
        HoodEncoder  = hardwareMap.get(AnalogInput.class, "hoodEncoder");
        HoodMotor = hardwareMap.get(CRServoImplEx.class, "HoodMotor");

        axonController = new RTPAxon(HoodMotor,HoodEncoder, RTPAxon.Direction.FORWARD);
        axonController.setPidCoeffs(0.009,0,0);
        axonController.setMaxPower(1);

    }

    @Override
    public void loop() {
        axonController.update();
        if(gamepad1.dpad_up){
            axonController.changeTargetRotation(0.5);
        }else if(gamepad1.dpad_down){
            axonController.changeTargetRotation(-0.5);
        }
        telemetry.addData("Motors", axonController.getTotalRotation());
    }
}
