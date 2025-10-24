package org.firstinspires.ftc.teamcode.lib.Subsystems.Turret;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import edu.wpi.first.math.controller.PIDController;

public class Turret extends WSubsystem {
    DcMotorEx turretMotor;
    WEncoder TurretEncoder;
    WActuatorGroup TurretController;

    @Override
    public void init(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class,"turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        TurretEncoder = new WEncoder(new MotorEx(hardwareMap, "turretMotor").encoder);

        TurretController = new WActuatorGroup(this::getTurretAngle,turretMotor)
                .setPIDController(new PIDController(0.015,0.09,0.001))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,0);
        TurretEncoder.encoder.reset();

    }
    public double getTurretAngle(){
        double encoderRots= TurretEncoder.getPosition()/28;
        return encoderRots*(1.0/(1+(46.0/11.0)))*(16.0/112.0)*360;
    }
    double handleTurretSetpoint(double setpoint){
        if(setpoint<-125){
            return -125;
        } else if (setpoint>125) {
            return 125;
        }
        return setpoint;
    }
    public void setTurretAngle(double Angle){
        TurretController.setTargetPosition(handleTurretSetpoint(Angle));
    }

    @Override
    public void periodic() {
        TurretController.setVoltageSupplier(Superstructure.voltage);
        TurretController.periodic();
    }

    @Override
    public void read() {
        TurretController.read();
    }

    @Override
    public void write() {
        TurretController.write();
    }

    @Override
    public void reset() {

    }
}
