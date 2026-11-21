package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//fiona ayc
@Config
@TeleOp(name = "MechCat Drive", group = "MechCat")
public class MechcatDriveTrain extends LinearOpMode {

    private static final double INTAKE_POWER = 0.40;

    // ===================== SHOOTER =====================
    // Enable ONLY after connecting and checking the shooter encoder at school.
    private static final boolean SHOOTER_ENCODER_CONFIRMED = false;
    private static final double SHOOTER_TICKS_PER_REV = 384.5;
    private static final double SHOOTER_RAMP_RPM_PER_SEC = 200.0;

    // Paste tuned values from pidTuner (units: ticks/sec). Editable in FTC Dashboard.
    public static double SHOOTER_kS = 0;
    public static double SHOOTER_kV = 0;
    public static double SHOOTER_kP = 0;
    public static double SHOOTER_kI = 0;
    public static double SHOOTER_kD = 0;
    public static double SHOOTER_INTEGRAL_ZONE = 200;

    // ===================== SLIDE =====================
    // Paste tuned values from SlideTuner (units: encoder ticks). Editable in FTC Dashboard.
    public static double SLIDE_kP = 0;
    public static double SLIDE_kI = 0;
    public static double SLIDE_kD = 0;
    public static double SLIDE_kG = 0;      // holds slide against gravity
    public static double SLIDE_kSync = 0;   // only for independent left/right slides
    public static double SLIDE_MAX_POWER = 1.0;

    // Positions in ticks. MEASURE these on your robot with SlideTuner!
    public static double SLIDE_MIN = 0;
    public static double SLIDE_LOW = 500;
    public static double SLIDE_MID = 1000;
    public static double SLIDE_HIGH = 1500;
    public static double SLIDE_MAX = 1600;  // just below the real top
    public static double SLIDE_MANUAL_SPEED = 1500; // ticks/sec at full stick

    // ===================== HOOD =====================
    // Never substitute arbitrary 0/1 endpoints. Calibrate with linkage detached.
    private static final boolean HOOD_CALIBRATED = false;
    private static final double HOOD_LEVEL_1 = Double.NaN;
    private static final double HOOD_LEVEL_5 = Double.NaN;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // ---- Drive + intake (names must match the robot configuration) ----
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");   // 左前
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");  // 右前
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");    // 左后
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");   // 右后
        DcMotor intake     = hardwareMap.get(DcMotor.class, "intake");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor[] allMotors = {frontLeft, frontRight, backLeft, backRight, intake};
        for (DcMotor motor : allMotors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // ---- Shooter (optional) ----
        DcMotorEx shooter = hardwareMap.tryGet(DcMotorEx.class, "shooter");
        if (shooter != null) {
            shooter.setPower(0);
            shooter.setDirection(DcMotorSimple.Direction.FORWARD); // must match pidTuner!
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // ---- Slide (optional): BOTH motors must be configured ----
        DcMotorEx leftSlide  = hardwareMap.tryGet(DcMotorEx.class, "ayc");
        DcMotorEx rightSlide = hardwareMap.tryGet(DcMotorEx.class, "fiona");


        boolean slideEnabled = leftSlide != null && rightSlide != null;
        if (slideEnabled) {
            leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);   // must match SlideTuner!
            rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);  // must match SlideTuner!
            for (DcMotorEx m : new DcMotorEx[]{leftSlide, rightSlide}) {
                m.setPower(0);
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // slide MUST be at bottom on INIT
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        // ---- Hood (optional) ----
        Servo hood = hardwareMap.tryGet(Servo.class, "hood");
        boolean hoodEnabled = hood != null && HOOD_CALIBRATED
                && Double.isFinite(HOOD_LEVEL_1) && Double.isFinite(HOOD_LEVEL_5)
                && HOOD_LEVEL_1 >= 0 && HOOD_LEVEL_1 <= 1
                && HOOD_LEVEL_5 >= 0 && HOOD_LEVEL_5 <= 1;
        int hoodLevel = 1;
        if (hoodEnabled) hood.setPosition(HOOD_LEVEL_1);

        telemetry.addLine("GAMEPAD 1:");
        telemetry.addLine("  Left stick: drive/strafe. Right stick: turn. Hold LB: slow.");
        telemetry.addLine("  RB: intake toggle. A: shooter toggle. D-pad L/R: shooter RPM.");
        telemetry.addLine("  D-pad up/down: hood levels 1-5.");
        telemetry.addLine("GAMEPAD 2:");
        telemetry.addLine("  D-pad down/left/right/up: slide bottom/low/mid/high.");
        telemetry.addLine("  Left stick: manual slide adjust.");
        telemetry.addData("Shooter enabled", shooter != null && SHOOTER_ENCODER_CONFIRMED);
        telemetry.addData("Slide enabled", slideEnabled);
        telemetry.addData("Hood enabled", hoodEnabled);
        telemetry.addLine("Make sure the slide is fully DOWN before INIT.");
        telemetry.update();
        waitForStart();

        // ---- Gamepad 1 state ----
        boolean intakeOn = false;
        boolean previousRightBumper = gamepad1.right_bumper;
        boolean shooterOn = false;
        boolean shooterFault = false;
        boolean previousA = gamepad1.a;
        boolean previousUp = gamepad1.dpad_up;
        boolean previousDown = gamepad1.dpad_down;
        boolean previousLeft = gamepad1.dpad_left;
        boolean previousRight = gamepad1.dpad_right;
        double targetRpm = 200.0; // Motor gearbox output RPM, NOT flywheel RPM.
        double commandedRpm = 0.0;
        double noFeedbackSince = -1.0;

        // ---- Shooter PID state ----
        double shooterTotalError = 0;
        double shooterPreviousError = 0;
        boolean shooterWasOn = false;

        // ---- Slide PID state ----
        double slideTarget = SLIDE_MIN;
        double slideTotalError = 0;
        double slidePreviousError = 0;

        ElapsedTime timer = new ElapsedTime();
        double previousTime = timer.seconds();

        try {
            while (opModeIsActive()) {
                double now = timer.seconds();
                double dt = Range.clip(now - previousTime, 0.0, 0.1);
                previousTime = now;
                TelemetryPacket packet = new TelemetryPacket();

                // ===================== DRIVE =====================
                double forward = deadzone(-gamepad1.left_stick_y);
                double strafe  = deadzone(gamepad1.left_stick_x);
                double turn    = deadzone(gamepad1.right_stick_x);

                double maxPower = gamepad1.left_bumper ? 0.20 : 0.40;
                double divisor = Math.max(1.0,
                        Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
                double scale = maxPower / divisor;

                frontLeft.setPower((forward + strafe + turn) * scale);
                frontRight.setPower((forward - strafe - turn) * scale);
                backLeft.setPower((forward - strafe + turn) * scale);
                backRight.setPower((forward + strafe - turn) * scale);

                // ===================== INTAKE =====================
                boolean rightBumper = gamepad1.right_bumper;
                if (rightBumper && !previousRightBumper) intakeOn = !intakeOn;
                previousRightBumper = rightBumper;
                intake.setPower(intakeOn ? INTAKE_POWER : 0.0);

                // ===================== SHOOTER =====================
                boolean a = gamepad1.a;
                if (a && !previousA && shooter != null
                        && SHOOTER_ENCODER_CONFIRMED && !shooterFault) {
                    shooterOn = !shooterOn;
                }
                previousA = a;
                if (gamepad1.dpad_right && !previousRight) targetRpm += 25;
                if (gamepad1.dpad_left && !previousLeft) targetRpm -= 25;
                targetRpm = Range.clip(targetRpm, 100.0, 350.0);
                previousRight = gamepad1.dpad_right;
                previousLeft = gamepad1.dpad_left;

                double actualRpm = 0;
                double shooterPower = 0;
                if (shooter != null && SHOOTER_ENCODER_CONFIRMED) {
                    double goal = shooterOn ? targetRpm : 0;
                    commandedRpm = shooterOn
                            ? Math.min(goal, commandedRpm + SHOOTER_RAMP_RPM_PER_SEC * dt) : 0;
                    double currentTps = shooter.getVelocity();
                    actualRpm = currentTps * 60.0 / SHOOTER_TICKS_PER_REV;

                    // Missing/reversed feedback or a stall latches a fault until restart.
                    if (shooterOn && commandedRpm >= 75 && actualRpm < 20) {
                        if (noFeedbackSince < 0) noFeedbackSince = now;
                        if (now - noFeedbackSince >= 0.75) {
                            shooterFault = true;
                            shooterOn = false;
                            commandedRpm = 0;
                        }
                    } else {
                        noFeedbackSince = -1;
                    }

                    double targetTps = commandedRpm * SHOOTER_TICKS_PER_REV / 60.0;
                    if (shooterOn && targetTps > 0) {
                        double error = targetTps - currentTps;
                        if (!shooterWasOn) shooterPreviousError = error;
                        if (Math.abs(error) < SHOOTER_INTEGRAL_ZONE) {
                            shooterTotalError += error * dt;
                        } else {
                            shooterTotalError = 0;
                        }
                        double derivative = dt > 0 ? (error - shooterPreviousError) / dt : 0;
                        shooterPreviousError = error;

                        double feedforward = SHOOTER_kS + SHOOTER_kV * targetTps;
                        double pid = SHOOTER_kP * error
                                + SHOOTER_kI * shooterTotalError
                                + SHOOTER_kD * derivative;
                        shooterPower = Range.clip(feedforward + pid, 0.0, 1.0);
                    } else {
                        shooterTotalError = 0;
                        shooterPreviousError = 0;
                    }
                    shooterWasOn = shooterOn;
                    shooter.setPower(shooterPower);

                    packet.put("Shooter target RPM", commandedRpm);
                    packet.put("Shooter actual RPM", actualRpm);
                    packet.put("Shooter power", shooterPower);
                }

                // ===================== SLIDE (gamepad 2) =====================
                double slidePosition = 0;
                if (slideEnabled) {
                    // Presets
                    if (gamepad2.dpad_down)  slideTarget = SLIDE_MIN;
                    if (gamepad2.dpad_left)  slideTarget = SLIDE_LOW;
                    if (gamepad2.dpad_right) slideTarget = SLIDE_MID;
                    if (gamepad2.dpad_up)    slideTarget = SLIDE_HIGH;

                    // Manual fine adjust: stick moves the TARGET, PID follows it
                    double manual = deadzone(-gamepad2.left_stick_y);
                    slideTarget += manual * SLIDE_MANUAL_SPEED * dt;
                    slideTarget = Range.clip(slideTarget, SLIDE_MIN, SLIDE_MAX);

                    int leftPos  = leftSlide.getCurrentPosition();
                    int rightPos = rightSlide.getCurrentPosition();
                    slidePosition = (leftPos + rightPos) / 2.0;

                    double error = slideTarget - slidePosition;
                    slideTotalError += error * dt;
                    double derivative = dt > 0 ? (error - slidePreviousError) / dt : 0;
                    slidePreviousError = error;

                    double slidePower = SLIDE_kP * error
                            + SLIDE_kI * slideTotalError
                            + SLIDE_kD * derivative
                            + SLIDE_kG;

                    // At the bottom: rest instead of fighting gravity
                    if (slideTarget <= SLIDE_MIN + 10 && slidePosition < SLIDE_MIN + 20) {
                        slidePower = 0;
                        slideTotalError = 0;
                    }
                    slidePower = Range.clip(slidePower, -SLIDE_MAX_POWER, SLIDE_MAX_POWER);

                    double syncError = leftPos - rightPos;   // + means left is higher
                    double correction = SLIDE_kSync * syncError;
                    leftSlide.setPower(Range.clip(slidePower - correction, -1, 1));
                    rightSlide.setPower(Range.clip(slidePower + correction, -1, 1));

                    packet.put("Slide target", slideTarget);
                    packet.put("Slide position", slidePosition);
                    packet.put("Slide sync error", syncError);
                    packet.put("Slide power", slidePower);
                }

                // ===================== HOOD =====================
                if (hoodEnabled) {
                    int oldLevel = hoodLevel;
                    if (gamepad1.dpad_up && !previousUp) hoodLevel++;
                    if (gamepad1.dpad_down && !previousDown) hoodLevel--;
                    hoodLevel = Math.max(1, Math.min(5, hoodLevel));
                    if (hoodLevel != oldLevel) {
                        double position = HOOD_LEVEL_1
                                + (HOOD_LEVEL_5 - HOOD_LEVEL_1) * (hoodLevel - 1) / 4.0;
                        hood.setPosition(position);
                    }
                }
                previousUp = gamepad1.dpad_up;
                previousDown = gamepad1.dpad_down;

                // ===================== TELEMETRY =====================
                dashboard.sendTelemetryPacket(packet);
                telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
                telemetry.addData("Shooter", shooterFault ? "FAULT: restart after checking encoder/stall"
                        : shooter == null ? "Not configured"
                        : !SHOOTER_ENCODER_CONFIRMED ? "Disabled: encoder not confirmed"
                        : shooterOn ? "ON" : "OFF");
                telemetry.addData("Motor target / measured RPM", "%.0f / %.0f", targetRpm, actualRpm);
                telemetry.addData("Slide", slideEnabled
                        ? String.format("target %.0f / pos %.0f", slideTarget, slidePosition)
                        : "Not configured");
                telemetry.addData("Hood", hoodEnabled ? "Level " + hoodLevel : "Disabled: not calibrated");
                telemetry.update();

                idle();
            }
        } finally {
            if (shooter != null) shooter.setPower(0);
            if (slideEnabled) {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }
            for (DcMotor motor : allMotors) motor.setPower(0);
        }
    }

    private double deadzone(double value) {
        return Math.abs(value) <= 0.05 ? 0.0 :
                Math.copySign((Math.abs(value) - 0.05) / 0.95, value);
    }
}