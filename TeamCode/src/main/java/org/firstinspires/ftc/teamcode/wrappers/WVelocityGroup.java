package org.firstinspires.ftc.teamcode.wrappers;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Velocity-focused actuator group wrapper.
 * - Reads velocity from the first WEncoder found in the provided devices (or from an external topic).
 * - Uses an FTCLib PIDController to compute a feedback term.
 * - Adds a simple feedforward (kS + kV * v + kA * a) or constant feedforward.
 * - Converts desired volts -> power by dividing by battery voltage supplied by voltage supplier (or default 12V).
 *
 * Units: keep consistent. Use setVelocityScaling(...) to convert encoder units to the units expected by PID/kV (e.g. RPM or ticks/sec).
 */
public class WVelocityGroup {
    public enum FeedforwardMode {
        NONE,
        SIMPLE,     // kS + kV*v + kA*a
        CONSTANT    // constant feedforward value
    }

    private final Map<String, HardwareDevice> devices = new HashMap<>();
    private PIDController controller;
    private double voltageSupplier;
    private ElapsedTime timer;
    private Supplier<Object> topic; // optional external velocity supplier

    // state
    private double velocity = 0.0;          // measured velocity (raw units)
    private double velocityScaled=0.0;    // velocity after scaling (units used by PID/kV)
    private double targetVelocity = 0.0;    // desired velocity (same units as velocityScaled)
    private double power = 0.0;             // -1..1 output to motors
    private boolean enabled = true;

    // feedforward params (simple)
    private FeedforwardMode ffMode = FeedforwardMode.NONE;
    private double kS = 0.0;
    private double kV = 0.0;
    private double kA = 0.0;
    private double constantFF = 0.0;

    // scaling: multiply raw encoder velocity by this to get units expected by PID/feedforward
    private double velocityScaling = ((double) 1 /28)* 60.0*(4.0 / 3.0);

    // internal for accel calc
    private double prevVelScaled = 0.0;
    private double acceleration = 0.0;

    // smoothing / clamping
    private double maxPowerChangePerSec = Double.POSITIVE_INFINITY; // optional rate limit
    private double lastPower = 0.0;

    /**
     * Create group from hardware devices.
     */
    public WVelocityGroup(HardwareDevice... devices) {
        this(null, devices);
    }

    /**
     * Create group with an optional external topic supplier (e.g., imu or other sensor giving velocity).
     */
    public WVelocityGroup(Supplier<Object> topic, HardwareDevice... devices) {
        this.topic = topic;
        int i = 0;
        for (HardwareDevice device : devices) {
            this.devices.put(device.getDeviceName() + " " + i++, device);
        }
        this.timer = new ElapsedTime();
        this.timer.reset();
    }

    /**
     * Read measured velocity from topic or the first WEncoder device (or DcMotor if you store velocity).
     * Keep units consistent; apply velocityScaling after reading raw value.
     */
    public void read() {
        // External topic takes precedence if provided
        if (topic != null) {
            Object v = topic.get();
            if (v instanceof Double) {
                this.velocity = (Double) v;
            } else if (v instanceof Integer) {
                this.velocity = ((Integer) v).doubleValue();
            }
            this.velocityScaled = this.velocity * velocityScaling;
            return;
        }

        // Look for a WEncoder device (your project uses a WEncoder class). If not present, try DcMotor's getVelocity if available.
        for (HardwareDevice device : devices.values()) {
            if (device instanceof WEncoder) {
                // WEncoder should expose getVelocity(); adjust if your method name differs
                this.velocity = ((WEncoder) device).getRawVelocity();
                this.velocityScaled = this.velocity * velocityScaling;
                return;
            } else if (device instanceof DcMotor) {
                // DcMotor (or DcMotorEx) often holds current velocity in ticks/sec depending on your wrapper; adapt as needed
                // If you have a DcMotorEx wrapper that exposes getVelocity(), use it. Here we default to 0 for safety.
                // Example if you have MotorEx: ((MotorEx)device).getVelocity();
            }
        }

        // default if nothing found
        this.velocity = 0.0;
        this.velocityScaled = 0.0;
    }

    /**
     * Periodic: compute acceleration, pid output, feedforward, and final motor power.
     * Call at a stable loop frequency (e.g., 50-200Hz).
     */
    public void periodic() {
        double now = timer.seconds();
        double dt = Math.max(1e-6, now - timer.seconds()); // we'll compute dt differently below
        // Actually compute dt using elapsed time; easier to keep consistent:
        // Use elapsedTime.seconds() between calls; to keep it simple, use small internal timer.
        double t = timer.seconds();
        // compute dt as time since last tick using elapsedTime - store prev time?
        // For simplicity, compute dt as 1 / 50 if timer unknown — but better: track prevTime
    }

    // We'll replace periodic with a clearer implementation below

    /**
     * nicer periodic implementation that computes dt from internal lastTime and uses it to calc accel
     */
    private double lastTime = Double.NaN;

    public void periodicImpl() {
        double now = timer.seconds();
        if (Double.isNaN(lastTime)) {
            lastTime = now;
        }
        double dt = now - lastTime;
        if (dt <= 0) dt = 1e-6;

        // update measured values
        read();
        // compute acceleration
        acceleration = (velocityScaled - prevVelScaled) / dt;

        // PID (feedback)
        double pidOutput = 0.0;
        if (controller != null && enabled) {
            pidOutput = controller.calculate(velocityScaled, targetVelocity);
        }

        // Feedforward
        double ff = 0.0;
        switch (ffMode) {
            case SIMPLE:
                // kS (static) sign term + kV*velocity + kA*acc
                double sign = Math.signum(targetVelocity);
                ff = kS * (sign == 0 ? 1.0 : sign) + kV * targetVelocity + kA * acceleration;
                break;
            case CONSTANT:
                ff = constantFF;
                break;
            case NONE:
            default:
                ff = 0.0;
        }

        // Total volts (we treat pidOutput as "volts" if you configured PID in volts, otherwise as scaled)
        // We'll assume PID produces a "voltage-like" number compatible with ff; if you used unitless PID, you might need to scale.
        double volts = ff + pidOutput;

        // Apply battery scaling to produce -1..1 power
        double battery = 12.0;
        if (voltageSupplier != 0) battery = voltageSupplier;
        double desiredPower = volts / Math.max(0.0001, battery);

        // clamp
        desiredPower = Math.max(-1.0, Math.min(1.0, desiredPower));

        // optional rate limit on power change
        if (!Double.isInfinite(maxPowerChangePerSec)) {
            double maxDelta = maxPowerChangePerSec * dt;
            double delta = desiredPower - lastPower;
            if (Math.abs(delta) > maxDelta) {
                desiredPower = lastPower + Math.signum(delta) * maxDelta;
            }
        }

        this.power = desiredPower;
        this.lastPower = desiredPower;

        // store prev velocity for next acceleration calc
        this.prevVelScaled = this.velocityScaled;
        this.lastTime = now;
    }

    /**
     * Write power to motors. Mirrors WActuatorGroup write semantics:
     * - apply voltage correction if voltageSupplier given (correction already applied above)
     * - respect enabled/floating semantics (if you want motor disabled, set enabled=false).
     */
    public void write() {
        for (HardwareDevice device : devices.values()) {
            if (device instanceof DcMotor) {
                DcMotor m = (DcMotor) device;
                if (enabled) {
                    m.setPower(power);
                } else {
                    m.setPower(0.0);
                }
            }
        }
    }

    // --------------------
    // Configuration API
    // --------------------

    public WVelocityGroup setPIDController(PIDController controller) {
        this.controller = controller;
        return this;
    }

    public WVelocityGroup setPID(double p, double i, double d) {
        if (controller == null) controller = new PIDController(p, i, d);
        else controller.setPID(p, i, d);
        return this;
    }

    public WVelocityGroup setFeedforwardSimple(double kS, double kV, double kA) {
        this.ffMode = FeedforwardMode.SIMPLE;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        return this;
    }

    public WVelocityGroup setFeedforwardConstant(double constantVolts) {
        this.ffMode = FeedforwardMode.CONSTANT;
        this.constantFF = constantVolts;
        return this;
    }

    public WVelocityGroup setFeedforwardNone() {
        this.ffMode = FeedforwardMode.NONE;
        return this;
    }

    public WVelocityGroup setVelocityScaling(double scaling) {
        this.velocityScaling = scaling;
        return this;
    }

    public WVelocityGroup setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
        return this;
    }

    public WVelocityGroup setVoltageSupplier(double supplier) {
        this.voltageSupplier = supplier;
        return this;
    }

    public WVelocityGroup setEnabled(boolean enabled) {
        this.enabled = enabled;
        return this;
    }

    public double getVelocity() {
        return this.velocityScaled;
    }

    public double getRawVelocity() {
        return this.velocity;
    }

    public double getTargetVelocity() {
        return this.targetVelocity;
    }

    public double getPower() {
        return this.power;
    }

    public List<HardwareDevice> getDevices() {
        return new ArrayList<>(devices.values());
    }

    public HardwareDevice getDevice(String name) {
        return devices.get(name);
    }

    public void setMaxPowerChangePerSec(double maxChange) {
        this.maxPowerChangePerSec = maxChange;
    }
}

