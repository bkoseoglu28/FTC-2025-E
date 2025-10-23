package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.MathUtils;
import org.firstinspires.ftc.teamcode.lib.RTPAxon;
import org.firstinspires.ftc.teamcode.lib.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.Sensors;
import org.firstinspires.ftc.teamcode.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.wrappers.WVelocityGroup;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.GoBildaPinpointDriver;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.util.Units;

@Config
@TeleOp(name = "SOLO")
public class SOLO extends OpMode {
    RobotHardware robot = RobotHardware.getInstance();
    double turretsetpoint=0;






    @Override
    public void init() {

        robot.init(hardwareMap);
        robot.reset();
        robot.read();

        turretsetpoint=0;
    }

    public void setFeedersMotor(double speed){
        robot.lFeederMotor.setPower(speed);
        robot.rFeederMotor.setPower(speed);
        robot.blFeederMotor.setPower(speed);
        robot.brFeederMotor.setPower(speed);
    }

    public void setRevolverMotor(double speed){

        robot.revolverMotor.setPower(speed);
    }
    public double handleTurretSetpoint(double setpoint){
        if(setpoint<-125){
            return -125;
        } else if (setpoint>125) {
            return 125;
        }
        return setpoint;
    }

    @Override
    public void loop() {
        robot.clearBulkCache();


        //RevolverController.write();
        double voltage=hardwareMap.voltageSensor.iterator().next().getVoltage();

        robot.VelocityController.setVoltageSupplier(voltage);
        robot.TurretController.setVoltageSupplier(voltage);
        robot.RevolverController.setVoltageSupplier(voltage);

        robot.drivetrain.set(-gamepad1.left_stick_x,gamepad1.left_stick_y, -gamepad1.right_stick_x,new Rotation2d(robot.getAngle()));
        if(robot.LL3.getLatestResult()!= null){
            LLResult result = robot.LL3.getLatestResult();
            double error =result.getTx();
            double setpoint=(robot.OdometryModule.getHeading(AngleUnit.DEGREES))+error;
            robot.TurretController.setTargetPosition(handleTurretSetpoint(setpoint));
        }
        robot.axonController.update();


        if(gamepad1.triangle){
            robot.VelocityController.setTargetVelocity(4500);
            //robot.TurretController.setTargetPosition(135);
            //RevolverController.setTargetPosition(120);
        }else if(gamepad1.square){
            robot.VelocityController.setTargetVelocity(2250);
            //robot.TurretController.setTargetPosition(-135);
            //RevolverController.setTargetPosition(-120);
        }
        else if(gamepad1.cross){
            robot.VelocityController.setTargetVelocity(0);
            //robot.TurretController.setTargetPosition(0);
            //RevolverController.setTargetPosition(0);
        } else if (gamepad1.right_bumper) {
            robot.VelocityController.setTargetVelocity(3500);
            robot.axonController.setTargetRotation(24);
            if((robot.axonController.getTotalRotation()>20)){
                robot.RevolverController.setTargetPosition(120);
                setFeedersMotor(1);
            }
        }else{
            robot.VelocityController.setTargetVelocity(0);
            robot.axonController.setTargetRotation(0);
            setFeedersMotor(0);
            robot.RevolverController.setTargetPosition(0);
        }

//        telemetry.addData("Target RPM",VelocityController.getTargetVelocity());
//        telemetry.addData("Measured RPM",VelocityController.getVelocity());
//        telemetry.addData("Shooter Velocity",upShooterMotor.getVelocity());
        telemetry.addData("Turret pose",robot.doubleSubscriber(Sensors.SensorType.TURRETENCODER));
        telemetry.addData("Revolver pose",robot.doubleSubscriber(Sensors.SensorType.REVOLVERENCODER));
        telemetry.addData("Flywheel vel",robot.VelocityController.getVelocity());
        telemetry.addData("x",robot.OdometryModule.getPose2d().getX());
        telemetry.addData("tx", robot.LL3.getLatestResult().getTx());
        telemetry.addData("y",robot.OdometryModule.getPose2d().getY());
        telemetry.addData("r",robot.OdometryModule.getHeading(AngleUnit.DEGREES));

        robot.periodic();
        robot.read();
        robot.write();
    }
}
