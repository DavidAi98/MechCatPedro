package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


/**
 * Two-motor lift: moves to a height (inches) at a controlled speed,
 * then HOLDS it (gravity feedforward + hold-only integral + battery compensation).
 * Every "public static" value is editable live in FTC Dashboard under "Slide".
 */
@Config
public class Slide {

    // ---- Units ----
    public static double TICKS_PER_INCH = 1;   // measure! (tuning step 1)

    // ---- Speed limits ----
    public static double MAX_VELOCITY = 20;    // inches/sec
    public static double MAX_ACCEL = 40;       // inches/sec²
    public static double MANUAL_SPEED = 8;     // inches/sec at full stick



    // ---- Gains (units: inches) ----
    public static double kG = 0;     // holds lift against gravity
    public static double kV = 0;     // power per (inch/sec)
    public static double kA = 0;     // usually 0
    public static double kP = 0;
    public static double kI = 0;     // removes leftover error while HOLDING
    public static double kD = 0;
    public static double kSync = 0;  // only for independent left/right sides

    // ---- Holding ----
    public static double INTEGRAL_ZONE = 1.0;  // I only works within this many inches
    public static double MAX_I_POWER = 0.15;   // I can never add more than this
    public static boolean VOLTAGE_COMP = true;
    public static double NOMINAL_VOLTAGE = 12.0;
    public static double MAX_POWER = 1.0;

    // ---- Heights (inches from bottom) ----
    public static double MIN = 0;
    public static double LOW = 5;
    public static double MID = 12;
    public static double HIGH = 20;
    public static double MAX = 22;   // just below the real top!

    // ---- Hardware (applied at INIT) ----
    public static String LEFT_NAME = "ayc";
    public static String RIGHT_NAME = "fiona";
    public static boolean LEFT_REVERSED = false;
    public static boolean RIGHT_REVERSED = true;

    private final DcMotorEx left, right;
    private final VoltageSensor battery;
    private final ElapsedTime timer = new ElapsedTime();

    private double goal = 0, setpoint = 0, setpointVel = 0, setpointAccel = 0;
    private double manualInput = 0;
    private double previousTime, previousError, totalError;
    private double position, velocity, power, syncError, iTerm;
    private double voltage = 12.0, lastVoltageRead = -1;

    public static boolean USE_MOTION_PROFILE = true; // false = full-power direct PID

    public static boolean SHOW_CURRENT = true; // turn off after tuning (slows the loop)

    public static Slide tryCreate(HardwareMap hardwareMap) {
        DcMotorEx l = hardwareMap.tryGet(DcMotorEx.class, LEFT_NAME);
        DcMotorEx r = hardwareMap.tryGet(DcMotorEx.class, RIGHT_NAME);
        if (l == null || r == null) return null;
        return new Slide(l, r, hardwareMap.voltageSensor.iterator().next());
    }
    public Slide(HardwareMap hardwareMap) {
        this(hardwareMap.get(DcMotorEx.class, LEFT_NAME),
                hardwareMap.get(DcMotorEx.class, RIGHT_NAME),
                hardwareMap.voltageSensor.iterator().next());
    }

    private Slide(DcMotorEx left, DcMotorEx right, VoltageSensor battery) {
        this.left = left;
        this.right = right;
        this.battery = battery;
        left.setDirection(LEFT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setDirection(RIGHT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // lift MUST be down at INIT
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        resetTimer();
    }

    // ---------------- Commands ----------------
    public void setTarget(double inches) { goal = Range.clip(inches, MIN, MAX); }
    public void goToBottom() { setTarget(MIN); }
    public void goToLow()    { setTarget(LOW); }
    public void goToMid()    { setTarget(MID); }
    public void goToHigh()   { setTarget(HIGH); }
    public void setManualInput(double stick) { manualInput = stick; }

    public void resetTimer() {
        timer.reset();
        previousTime = 0;
        previousError = 0;
        totalError = 0;
        lastVoltageRead = -1;
    }

    // ---------------- Call every loop ----------------
    public void update() {
        double now = timer.seconds();
        double dt = Range.clip(now - previousTime, 0.0, 0.1);
        previousTime = now;
        if (dt <= 0) return;

        goal = Range.clip(goal + manualInput * MANUAL_SPEED * dt, MIN, MAX);

        // ---- Motion profile (or direct PID) ----
        if (USE_MOTION_PROFILE) {
            // Controlled speed: move the setpoint toward the goal smoothly
            double distance = goal - setpoint;
            double stoppingSpeed = Math.sqrt(2 * MAX_ACCEL * Math.abs(distance));
            double desiredVel = Math.signum(distance) * Math.min(MAX_VELOCITY, stoppingSpeed);
            double maxChange = MAX_ACCEL * dt;
            double newVel = setpointVel + Range.clip(desiredVel - setpointVel, -maxChange, maxChange);
            setpointAccel = (newVel - setpointVel) / dt;
            setpointVel = newVel;
            setpoint += setpointVel * dt;

            boolean passed = (distance > 0 && setpoint > goal) || (distance < 0 && setpoint < goal);
            if (passed || (Math.abs(goal - setpoint) < 0.05 && Math.abs(setpointVel) < 0.5)) {
                setpoint = goal;
                setpointVel = 0;
                setpointAccel = 0;
            }
        } else {
            // Direct PID (like OuttakeControl): jump straight to the goal, full power
            setpoint = goal;
            setpointVel = 0;
            setpointAccel = 0;
        }
        // "Holding" = profile finished AND actually close to the goal
        boolean holding = !isMoving() && Math.abs(goal - position) < INTEGRAL_ZONE;

        // ---- Measure ----
        int leftPos = left.getCurrentPosition();
        int rightPos = right.getCurrentPosition();
        position = (leftPos + rightPos) / 2.0 / TICKS_PER_INCH;
        velocity = (left.getVelocity() + right.getVelocity()) / 2.0 / TICKS_PER_INCH;

        // Battery voltage (read 4x/sec: reading every loop is slow)
        if (lastVoltageRead < 0 || now - lastVoltageRead > 0.25) {
            double v = battery.getVoltage();
            if (v > 7) voltage = v;   // ignore bad readings
            lastVoltageRead = now;
        }

        // ---- PID ----
        double error = setpoint - position;

        // I only builds while HOLDING and close to target -> no windup during moves.
        // While moving, the learned value is kept (gravity is the same at every height).
        if (holding && Math.abs(error) < INTEGRAL_ZONE) {
            totalError += error * dt;
        }
        if (kI > 0) {   // anti-windup: I can never exceed MAX_I_POWER
            double limit = MAX_I_POWER / kI;
            totalError = Range.clip(totalError, -limit, limit);
        }
        iTerm = kI * totalError;

        double derivative = (error - previousError) / dt;
        previousError = error;

        power = kG + kV * setpointVel + kA * setpointAccel
                + kP * error + iTerm + kD * derivative;

        // Battery compensation: same force at 13.5V or 11.8V
        if (VOLTAGE_COMP) power *= NOMINAL_VOLTAGE / voltage;

        // At the bottom: rest on the hard stop instead of fighting gravity
        if (goal <= MIN + 0.1 && setpoint <= MIN + 0.1 && position < MIN + 0.25) {
            power = 0;
            totalError = 0;
        }
        power = Range.clip(power, -MAX_POWER, MAX_POWER);

        // ---- Keep both sides level ----
        syncError = (leftPos - rightPos) / TICKS_PER_INCH;
        double correction = kSync * syncError;
        left.setPower(Range.clip(power - correction, -1, 1));
        right.setPower(Range.clip(power + correction, -1, 1));
    }
    public double leftPosInches()  { return left.getCurrentPosition()  / TICKS_PER_INCH; }
    public double rightPosInches() { return right.getCurrentPosition() / TICKS_PER_INCH; }

    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }

    public void addTelemetry(TelemetryPacket packet) {
        packet.put("Slide goal", goal);
        packet.put("Slide setpoint", setpoint);
        packet.put("Slide position", position);
        packet.put("Slide hold error", goal - position);
        packet.put("Slide planned speed", setpointVel);
        packet.put("Slide actual speed", velocity);
        packet.put("Slide I power", iTerm);
        packet.put("Slide power", power);
        packet.put("Slide sync error", syncError);
        packet.put("Battery V", voltage);

        packet.put("Slide left pos", leftPosInches());
        packet.put("Slide right pos", rightPosInches());
        if (SHOW_CURRENT) {
            packet.put("Left amps", left.getCurrent(CurrentUnit.AMPS));
            packet.put("Right amps", right.getCurrent(CurrentUnit.AMPS));
        }
    }

    public double getTarget()   { return goal; }
    public double getPosition() { return position; }
    public boolean isMoving()   { return setpoint != goal; }
    public boolean atTarget(double toleranceInches) {
        return !isMoving() && Math.abs(goal - position) < toleranceInches;
    }
}
