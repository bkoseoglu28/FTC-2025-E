package org.firstinspires.ftc.teamcode.lib.Subsystems.Revolver;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;

public class Revolver extends WSubsystem {
    DcMotorEx revolverMotor;
    WEncoder RevolverEncoder;
    WActuatorGroup RevolverController;

    @Override
    public void init(HardwareMap hardwareMap) {
        revolverMotor = hardwareMap.get(DcMotorEx.class,"revolverMotor");
        revolverMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        revolverMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RevolverEncoder = new WEncoder(new MotorEx(hardwareMap,"revolverMotor").encoder);
        RevolverController = new WActuatorGroup(()-> getRevolverAngle().getDegrees(),revolverMotor)
                .setPIDController(new edu.wpi.first.math.controller.PIDController(0.07,0,0.001))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,0.2)
                .setErrorTolerance(0.1)
                .enableContinuousInput(180,-180);
        RevolverController.setMaxPower(0.5);
        RevolverEncoder.encoder.reset();
    }
    public Rotation2d getRevolverAngle(){
        double encoderRots= RevolverEncoder.getPosition()/28;

        return new Rotation2d(encoderRots*(1.0/(((1+(46.0/17.0))) * (1+(46.0/17.0))))*(16.0/30.0)*2*Math.PI);
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
