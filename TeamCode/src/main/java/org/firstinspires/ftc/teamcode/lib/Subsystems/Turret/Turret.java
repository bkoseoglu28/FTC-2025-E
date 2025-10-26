package org.firstinspires.ftc.teamcode.lib.Subsystems.Turret;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.lib.ProfileConstraints;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.Util;
@Config
public class Turret extends WSubsystem {
    public static double kp;
    public static double ki;
    public static double kd;

    DcMotorEx turretMotor;
    WEncoder TurretEncoder;
    WActuatorGroup TurretController;

    @Override
    public void init(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class,"turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        TurretEncoder = new WEncoder(new MotorEx(hardwareMap, "turretMotor").encoder);

        TurretController = new WActuatorGroup(this::getTurretAngle,turretMotor)
                .setPIDController(new PIDController(0.008,0,0.0003))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,0);
        TurretEncoder.encoder.reset();


    }
    public boolean IsAtSetpoint(){
        return Util.epsilonEquals(getTurretAngle(),TurretController.getTargetPosition(),10);
    }
    public double ff(){
        Twist2d velocity = new Twist2d(Superstructure.drivetrain.OdometryModule.getVelX(DistanceUnit.METER),Superstructure.drivetrain.OdometryModule.getVelY(DistanceUnit.METER),Superstructure.drivetrain.OdometryModule.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        // Angular velocity component from tangential robot motion about the goal.
        double tangential_component = 0;//mLatestAimingParameters.get().getRobotToGoalRotation().sin() * velocity.dx / mLatestAimingParameters.get().getRange();
        double angular_component = Units.radiansToDegrees(velocity.dtheta);
        // Add (opposite) of tangential velocity about goal + angular velocity in local frame.
        return  -(angular_component + tangential_component);
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
        //TurretController.setPID(kp,ki,kd);
        TurretController.setVoltageSupplier(Superstructure.voltage);
        //TurretController.setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,ff());
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
