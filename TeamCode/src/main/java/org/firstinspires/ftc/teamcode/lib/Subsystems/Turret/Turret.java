package org.firstinspires.ftc.teamcode.lib.Subsystems.Turret;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.lib.ProfileConstraints;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.Util;

public class Turret extends WSubsystem {
//    public static double kp=0;
//    public static double ki=0;
//    public static double kd=0;
//    public static double feedforward=0;

    DcMotorEx turretMotor;
    WEncoder TurretEncoder;
    WActuatorGroup TurretController;
    PIDController visionPID;

    @Override
    public void init(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class,"turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TurretEncoder = new WEncoder(new MotorEx(hardwareMap, "turretMotor").encoder);

        TurretController = new WActuatorGroup(this::getTurretAngle,turretMotor)
                .setPIDController(new PIDController(0.0125,0.006,0.00016))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,0.04);
        TurretEncoder.encoder.reset();
        visionPID = new PIDController(0,0,0);
    }
    public boolean IsAtSetpoint(){
        return Util.epsilonEquals(getTurretAngle(),TurretController.getTargetPosition(),10);
    }
//    public double ff(){
//        Twist2d velocity = new Twist2d(Superstructure.drivetrain.OdometryModule.getVelX(DistanceUnit.METER),Superstructure.drivetrain.OdometryModule.getVelY(DistanceUnit.METER),Superstructure.drivetrain.OdometryModule.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
//        // Angular velocity component from tangential robot motion about the goal.
//        double tangential_component = 0;//mLatestAimingParameters.get().getRobotToGoalRotation().sin() * velocity.dx / mLatestAimingParameters.get().getRange();
//        double angular_component = Units.radiansToDegrees(velocity.dtheta);
//        // Add (opposite) of tangential velocity about goal + angular velocity in local frame.
//        return  -(angular_component + tangential_component);
//    }
    public double getTurretAngle(){
        double encoderRots= TurretEncoder.getPosition()/28;
        return encoderRots*(1.0/(((1+(46.0/17.0))) * (1+(46.0/17.0))))*(16.0/112.0)*360;
    }
//    public double getTurretFieldAngle(){
//        return getTurretAngle()-Superstructure.drivetrain.OdometryModule.getHeading(AngleUnit.DEGREES);
//    }
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
//        TurretController.setPID(kp,ki,kd);
//        TurretController.setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,feedforward);
        TurretController.periodic();
    }
    public void setVoltage(DoubleSupplier voltage){
        TurretController.setVoltageSupplier(voltage);
    }

    @Override
    public void read() {
        TurretController.read();
    }

    @Override
    public void write() {
//        if(Superstructure.vision.tv){
//            turretMotor.setPower(visionPID.calculate(Superstructure.vision.tx,0));
//        }else{
//            TurretController.write();
//        }
        TurretController.write();
    }

    @Override
    public void reset() {

    }
}
