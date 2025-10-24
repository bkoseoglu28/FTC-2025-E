package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.lib.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.wrappers.WVelocityGroup;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PinPointPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.GoBildaPinpointDriver;

@Config
public class RobotHardware {

    //drivetrain
    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackLeftMotor;
    public DcMotorEx dtBackRightMotor;

    public GoBildaPinpointDriver OdometryModule;
    public Limelight3A LL3;
    public PinPointPoseEstimator poseEstimator;


    // Shooter
    //public RevColorSensorV3 IntakeBanner;
    //public RevTouchSensor ElevatorLimit;
    public CRServo lFeederMotor;
    public CRServo rFeederMotor;
    public CRServo blFeederMotor;
    public CRServo brFeederMotor;
    public DcMotorEx revolverMotor;
    CRServoImplEx HoodMotor;
    AnalogInput HoodEncoder;
    public org.firstinspires.ftc.teamcode.lib.RTPAxon axonController;
    DcMotorEx masterShooterMotor;
    DcMotorEx slaveShooterMotor;
    DcMotorEx turretMotor;
    public WVelocityGroup VelocityController;
    public WActuatorGroup TurretController;
    public WActuatorGroup RevolverController;
    WEncoder orso;
    WEncoder TurretEncoder;
    WEncoder RevolverEncoder;
    int tickPerRev = 28;
    double targetRPM = 0.0;
    double shooterGearRatio = 4.0 / 3.0;

    public RevBlinkinLedDriver LEDS;


    private HardwareMap hardwareMap;

    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;

    private static RobotHardware instance = null;
    private boolean enabled;


    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;
    public LynxModule EXPENSION_HUB;


    private ArrayList<WSubsystem> subsystems;

    public MecanumDrivetrain drivetrain;
    public Vision vision;

    private final Object imuLock = new Object();
    public HashMap<Sensors.SensorType, Object> values;


    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    /**
     * Created at the start of every OpMode.
     *
     * @param hardwareMap The HardwareMap of the robot, storing all hardware devices
     */
    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.values = new HashMap<>();
        values.put(Sensors.SensorType.TURRETENCODER, 0.0);
        values.put(Sensors.SensorType.orospucocu, 0);
        values.put(Sensors.SensorType.POD_LEFT, 0.0);
        values.put(Sensors.SensorType.POD_FRONT, 0.0);
        values.put(Sensors.SensorType.REVOLVERENCODER, 0.0);
        values.put(Sensors.SensorType.HOODENCODER, 0.0);
        //values.put(Sensors.SensorType.ELEVATORLIMIT, false);
        lFeederMotor = hardwareMap.get(CRServo.class,"lFeederMotor");
        lFeederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rFeederMotor = hardwareMap.get(CRServo.class,"rFeederMotor");
        rFeederMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blFeederMotor = hardwareMap.get(CRServo.class, "blFeederMotor");
        blFeederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brFeederMotor = hardwareMap.get(CRServo.class, "brFeederMotor");
        brFeederMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        revolverMotor = hardwareMap.get(DcMotorEx.class,"revolverMotor");
        revolverMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        revolverMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretMotor = hardwareMap.get(DcMotorEx.class,"turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        HoodEncoder  = hardwareMap.get(AnalogInput.class, "hoodEncoder");
        HoodMotor = hardwareMap.get(CRServoImplEx.class, "HoodMotor");

        axonController = new org.firstinspires.ftc.teamcode.lib.RTPAxon(HoodMotor,HoodEncoder, org.firstinspires.ftc.teamcode.lib.RTPAxon.Direction.FORWARD);
        axonController.setPidCoeffs(0.009,0,0);

        masterShooterMotor = hardwareMap.get(DcMotorEx.class,"masterShooterMotor");
        slaveShooterMotor = hardwareMap.get(DcMotorEx.class,"slaveShooterMotor");
        masterShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slaveShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        masterShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slaveShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        orso = new WEncoder(new MotorEx(hardwareMap, "slaveShooterMotor").encoder);

        TurretEncoder = new WEncoder(new MotorEx(hardwareMap, "turretMotor").encoder);
        RevolverEncoder = new WEncoder(new MotorEx(hardwareMap,"revolverMotor").encoder);

        VelocityController = new WVelocityGroup(orso,masterShooterMotor,slaveShooterMotor)
                .setPIDController(new PIDController(0,0,0))
                .setFeedforwardSimple(1.5,0.0026,0.0001);
        TurretController = new WActuatorGroup(this::getTurretAngle,turretMotor)
                .setPIDController(new edu.wpi.first.math.controller.PIDController(0.015,0.09,0.001))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,0);
        RevolverController = new WActuatorGroup(()-> getRevolverAngle().getDegrees(),revolverMotor)
                .setPIDController(new edu.wpi.first.math.controller.PIDController(0.07,0,0.001))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,0.2)
                .setErrorTolerance(0.1)
                .enableContinuousInput(180,-180);
        TurretEncoder.encoder.reset();
        RevolverEncoder.encoder.reset();

        RevolverController.setMaxPower(0.5);

        this.OdometryModule= hardwareMap.get(GoBildaPinpointDriver.class,"LocalizerModule");
        OdometryModule.initialize();
        OdometryModule.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        OdometryModule.recalibrateIMU();
        OdometryModule.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        OdometryModule.setOffsets(-159.5,27,DistanceUnit.MM);


        this.dtBackLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.dtFrontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtFrontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.dtBackRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.dtFrontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtFrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LL3 = hardwareMap.get(Limelight3A.class,"limelight");
        LL3.pipelineSwitch(0);
        LL3.start();

        poseEstimator=new PinPointPoseEstimator(OdometryModule, VecBuilder.fill(0.3,0.3,0.3),VecBuilder.fill(0.1,0.1,0.1));



        //IntakeBanner = hardwareMap.get(RevColorSensorV3.class,"intakeSensor");
        //ElevatorLimit = hardwareMap.get(RevTouchSensor.class,"elevatorLimitSwitch");


        this.LEDS = hardwareMap.get(RevBlinkinLedDriver.class, "ledDriver");


//        this.IntakePivotLeft = new WServo(hardwareMap.get(Servo.class, "leftIntakeServo"));
//        this.IntakePivotRight = new WServo(hardwareMap.get(Servo.class, "rightIntakeServo"));
//        IntakePivotRight.setDirection(Servo.Direction.REVERSE);

//        this.intakePivotActuator = new WActuatorGroup(IntakePivotLeft, IntakePivotRight);


        modules = hardwareMap.getAll(LynxModule.class);


        for (LynxModule m : modules) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (m.isParent() && LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber())) CONTROL_HUB = m;
            if (!m.isParent())EXPENSION_HUB= m;
        }


        subsystems = new ArrayList<>();
        drivetrain = new MecanumDrivetrain();
        vision= new Vision();

                voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void read() {
        // Read all hardware devices here
//        if (Globals.CurrentPosition== Globals.ScoringPositions.LOW && ElevatorEncoder.getPosition() !=0&&ElevatorLimit.isPressed()){
//            ElevatorEncoder.encoder.reset();
//        }
//        if(SliderLimit.isPressed()){
//            SliderEncoder.encoder.reset();
//        }

        values.put(Sensors.SensorType.TURRETENCODER, getTurretAngle());
        values.put(Sensors.SensorType.REVOLVERENCODER, getRevolverAngle().getDegrees());
        values.put(Sensors.SensorType.orospucocu, orso.getRawVelocity());
        //values.put(Sensors.SensorType.INTAKEBANNER, IntakeBanner.getDistance(DistanceUnit.MM));
        //values.put(Sensors.SensorType.ELEVATORLIMIT, ElevatorLimit.isPressed());
        VelocityController.read();
        TurretController.read();
        RevolverController.read();
        vision.read();
    }

    public void write() {
        drivetrain.write();
        VelocityController.write();
        TurretController.write();
        RevolverController.write();
        vision.write();
    }

    public void periodic() {
//        if (voltageTimer.seconds() > 5) {
//            voltageTimer.reset();
//            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//        }
        poseEstimator.update();
        OdometryModule.update();
        drivetrain.periodic();
        VelocityController.periodicImpl();
        TurretController.periodic();
        RevolverController.periodic();
        vision.periodic();
//        if(Globals.IS_CLIMBING && Globals.SCORE){
//            elevatorActuator.setPID(10,0,0);
//            elevatorActuator.setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,1);
//        }else{
//            elevatorActuator.setPID(0.0018, 0.137, 0.00009);
//            elevatorActuator.setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT,0);
//        }

    }


    public double getAngle() {
        return OdometryModule.getHeading(AngleUnit.RADIANS);
    }

    public double getShooterRPM(){
        return orso.getRawVelocity();
    }
    public double getTurretAngle(){
        double encoderRots= TurretEncoder.getPosition()/28;
        return encoderRots*(1.0/(1+(46.0/11.0)))*(16.0/112.0)*360;
    }
    public Rotation2d getRevolverAngle(){
        double encoderRots= RevolverEncoder.getPosition()/28;

        return new Rotation2d(encoderRots*(1.0/(((1+(46.0/17.0))) * (1+(46.0/17.0))))*(16.0/30.0)*2*Math.PI);
    }

    public void reset() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.reset();
        }

        OdometryModule.resetPosAndIMU();
        if(vision.robotPose!=null){
            OdometryModule.resetPose(vision.robotPose);
        }else{
            OdometryModule.resetPose(new Pose2d());
        }

        OdometryModule.recalibrateIMU();
        vision.reset();


    }


    public void clearBulkCache() {
        CONTROL_HUB.clearBulkCache();
        EXPENSION_HUB.clearBulkCache();
    }

    public void addSubsystem(WSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }


    public double getVoltage() {
        return voltage;
    }

    public double doubleSubscriber(Sensors.SensorType topic) {
        Object value = values.getOrDefault(topic, 0.0);
        if (value instanceof Integer) {
            return ((Integer) value).doubleValue();
        } else if (value instanceof Double) {
            return (Double) value;
        } else {
            throw new ClassCastException();
        }
    }

    public int intSubscriber(Sensors.SensorType topic) {
        Object value = values.getOrDefault(topic, 0);
        if (value instanceof Integer) {
            return (Integer) value;
        } else if (value instanceof Double) {
            return ((Double) value).intValue();
        } else {
            throw new ClassCastException();
        }
    }

    public boolean booleanSubscriber(Sensors.SensorType topic) {
        return (boolean) values.getOrDefault(topic, 0);
    }

    public void kill() {
        instance = null;
    }

}
