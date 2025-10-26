package org.firstinspires.ftc.teamcode.lib.Subsystems.FlyWheel;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.wrappers.WVelocityGroup;

import edu.wpi.first.util.Util;

public class Flywheel extends WSubsystem {
    DcMotorEx masterShooterMotor;
    DcMotorEx slaveShooterMotor;
     WVelocityGroup VelocityController;
    WEncoder encoder;
    @Override
    public void init(HardwareMap hardwareMap) {
        masterShooterMotor = hardwareMap.get(DcMotorEx.class,"masterShooterMotor");
        slaveShooterMotor = hardwareMap.get(DcMotorEx.class,"slaveShooterMotor");
        masterShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slaveShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        masterShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slaveShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        encoder = new WEncoder(new MotorEx(hardwareMap, "slaveShooterMotor").encoder);
        VelocityController = new WVelocityGroup(encoder,masterShooterMotor,slaveShooterMotor)
                .setPIDController(new PIDController(0,0,0))
                .setFeedforwardSimple(1.4,0.00185,0.0003);
        //1.5,0.0026,0.0001
    }
    public boolean IsAtSetpoint(){
        return Util.epsilonEquals(getShooterRPM(),VelocityController.getTargetVelocity(),100);
    }
    public void setSetpointRPM(double rpm){
        VelocityController.setTargetVelocity(rpm);
    }
    public double getShooterRPM(){
        return VelocityController.getVelocity();
    }
    @Override
    public void periodic() {
        VelocityController.setVoltageSupplier(Superstructure.voltage);
        VelocityController.periodicImpl();
    }

    @Override
    public void read() {
        VelocityController.read();
    }

    @Override
    public void write() {
        VelocityController.write();
    }

    @Override
    public void reset() {

    }
}
