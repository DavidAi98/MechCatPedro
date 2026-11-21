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

    // =====================================================================
    // ======================  ADJUST SLIDE HERE  ==========================
    // =====================================================================
    // All values in encoder ticks. Also editable live in FTC Dashboard
    // (192.168.43.1:8080/dash -> Configuration -> MechcatDriveTrain).

    // -- Speed while holding the trigger --
    public static double SLIDE_UP_SPEED = 900;    // ticks/sec with LT fully pressed (537.7 ticks = 1 motor turn)
    public static double SLIDE_DOWN_SPEED = 905;  // ticks/sec with RT fully pressed
    public static double SLIDE_MAX_LEAD = 100;    // how far the target may run ahead of the slide

    // -- Safety limits --
    public static double SLIDE_MIN = 0;     // bottom
    public static double SLIDE_MAX = 300;  // never goes above this (set just below the real top!)

    // -- Strength --
    public static double SLIDE_MAX_POWER = 0.9;  // max motor power (0-1)

    // -- Control gains --
    public static double SLIDE_kG = 0.01;   // holds slide up against gravity. Sinks? raise. Creeps up? lower.
    public static double SLIDE_kP = 0.006;  // how hard it pushes toward the target. Slow? raise. Bounces? lower.
    public static double SLIDE_kD = 0;      // smooths the stop. Overshoots? add a little (0.0002).
    public static double SLIDE_kI = 0;      // usually leave 0
    public static double SLIDE_kSync = 0;   // only if one side lags the other

    // =====================================================================

    // ===================== DRIVE =====================
    public static double DRIVE_MAX_POWER = 0.40;  // normal speed
    public static double DRIVE_SLOW_POWER = 0.20; // while holding A
    public static double TURN_SPEED = 0.6;        // bumper turn speed (0-1)

    // [OFF] private static final double INTAKE_POWER = 0.40;

    // ===================== SHOOTER [OFF] =====================
    // [OFF] private static final boolean SHOOTER_ENCODER_CONFIRMED = false;
    // [OFF] private static final double SHOOTER_TICKS_PER_REV = 384.5;
    // [OFF] private static final double SHOOTER_RAMP_RPM_PER_SEC = 200.0;
    // [OFF] public static double SHOOTER_kS = 0;
    // [OFF] public static double SHOOTER_kV = 0;
    // [OFF] public static double SHOOTER_kP = 0;
    // [OFF] public static double SHOOTER_kI = 0;
    // [OFF] public static double SHOOTER_kD = 0;
    // [OFF] public static double SHOOTER_INTEGRAL_ZONE = 200;

    // ===================== HOOD =====================
    // Never substitute arbitrary 0/1 endpoints. Calibrate with linkage detached.
    private static final boolean HOOD_CALIBRATED = false;
    private static final double HOOD_LEVEL_1 = Double.NaN;
    private static final double HOOD_LEVEL_5 = Double.NaN;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // ---- Drive (names must match the robot configuration) ----
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");   // 左前
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");  // 右前
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");    // 左后
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");   // 右后
        // [OFF] DcMotor intake     = hardwareMap.get(DcMotor.class, "intake");

        // These directions must be the SAME in every OpMode (including autonomous)!
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // [OFF] intake.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor[] allMotors = {frontLeft, frontRight, backLeft, backRight}; // [OFF] intake removed
        for (DcMotor motor : allMotors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // ---- Shooter [OFF] ----
        // [OFF] DcMotorEx shooter = hardwareMap.tryGet(DcMotorEx.class, "shooter");
        // [OFF] if (shooter != null) {
        // [OFF]     shooter.setPower(0);
        // [OFF]     shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        // [OFF]     shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // [OFF]     shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // [OFF]     shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // [OFF] }

        // ---- Slide (optional): BOTH motors must be configured ----
        DcMotorEx leftSlide  = hardwareMap.tryGet(DcMotorEx.class, "ayc");
        DcMotorEx rightSlide = hardwareMap.tryGet(DcMotorEx.class, "fiona");

        boolean slideEnabled = leftSlide != null && rightSlide != null;
        if (slideEnabled) {
            leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
            rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
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

        telemetry.addLine("CONTROLS (gamepad 1):");
        telemetry.addLine("  Left stick Y: forward/back. Right stick X: strafe.");
        telemetry.addLine("  LB: turn clockwise. RB: turn counterclockwise. Hold A: slow.");
        telemetry.addLine("  Hold LT: slide up. Hold RT: slide down. Release: slide holds.");
        telemetry.addLine("  D-pad up/down: hood levels 1-5.");
        // [OFF] telemetry.addLine("  Intake toggle / shooter toggle: (off)");
        // [OFF] telemetry.addData("Shooter enabled", shooter != null && SHOOTER_ENCODER_CONFIRMED);
        telemetry.addData("Slide enabled", slideEnabled ? "YES" : "NO - check config names 'ayc' / 'fiona'");
        telemetry.addData("Hood enabled", hoodEnabled);
        telemetry.addLine("Make sure the slide is fully DOWN before INIT.");
        telemetry.update();
        waitForStart();

        // ---- Gamepad state ----
        // [OFF] boolean intakeOn = false;
        // [OFF] boolean previousRightBumper = gamepad1.right_bumper;
        // [OFF] boolean shooterOn = false;
        // [OFF] boolean shooterFault = false;
        // [OFF] boolean previousA = gamepad1.a;
        boolean previousUp = gamepad1.dpad_up;      // used by hood
        boolean previousDown = gamepad1.dpad_down;  // used by hood
        // [OFF] boolean previousLeft = gamepad1.dpad_left;
        // [OFF] boolean previousRight = gamepad1.dpad_right;
        // [OFF] double targetRpm = 200.0;
        // [OFF] double commandedRpm = 0.0;
        // [OFF] double noFeedbackSince = -1.0;

        // ---- Shooter PID state [OFF] ----
        // [OFF] double shooterTotalError = 0;
        // [OFF] double shooterPreviousError = 0;
        // [OFF] boolean shooterWasOn = false;

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
                // Left stick Y: forward/back. Right stick X: strafe.
                // LB: turn clockwise. RB: turn counterclockwise. Hold A: slow.
                double forward = deadzone(gamepad1.left_stick_y);
                double strafe  = deadzone(gamepad1.right_stick_x);

                double turn = 0;
                if (gamepad1.left_bumper)  turn += TURN_SPEED;   // clockwise
                if (gamepad1.right_bumper) turn -= TURN_SPEED;   // counterclockwise

                double maxPower = gamepad1.a ? DRIVE_SLOW_POWER : DRIVE_MAX_POWER;
                double divisor = Math.max(1.0,
                        Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
                double scale = maxPower / divisor;

                frontLeft.setPower((forward + strafe + turn) * scale);
                frontRight.setPower((forward - strafe - turn) * scale);
                backLeft.setPower((forward - strafe + turn) * scale);
                backRight.setPower((forward + strafe - turn) * scale);

                // ===================== INTAKE [OFF] =====================
                // [OFF] boolean rightBumper = gamepad1.right_bumper;
                // [OFF] if (rightBumper && !previousRightBumper) intakeOn = !intakeOn;
                // [OFF] previousRightBumper = rightBumper;
                // [OFF] intake.setPower(intakeOn ? INTAKE_POWER : 0.0);

                // ===================== SHOOTER [OFF] =====================
                // [OFF] boolean a = gamepad1.a;
                // [OFF] if (a && !previousA && shooter != null
                // [OFF]         && SHOOTER_ENCODER_CONFIRMED && !shooterFault) {
                // [OFF]     shooterOn = !shooterOn;
                // [OFF] }
                // [OFF] previousA = a;
                // [OFF] if (gamepad1.dpad_right && !previousRight) targetRpm += 25;
                // [OFF] if (gamepad1.dpad_left && !previousLeft) targetRpm -= 25;
                // [OFF] targetRpm = Range.clip(targetRpm, 100.0, 350.0);
                // [OFF] previousRight = gamepad1.dpad_right;
                // [OFF] previousLeft = gamepad1.dpad_left;
                // [OFF]
                // [OFF] double actualRpm = 0;
                // [OFF] double shooterPower = 0;
                // [OFF] if (shooter != null && SHOOTER_ENCODER_CONFIRMED) {
                // [OFF]     double goal = shooterOn ? targetRpm : 0;
                // [OFF]     commandedRpm = shooterOn
                // [OFF]             ? Math.min(goal, commandedRpm + SHOOTER_RAMP_RPM_PER_SEC * dt) : 0;
                // [OFF]     double currentTps = shooter.getVelocity();
                // [OFF]     actualRpm = currentTps * 60.0 / SHOOTER_TICKS_PER_REV;
                // [OFF]
                // [OFF]     if (shooterOn && commandedRpm >= 75 && actualRpm < 20) {
                // [OFF]         if (noFeedbackSince < 0) noFeedbackSince = now;
                // [OFF]         if (now - noFeedbackSince >= 0.75) {
                // [OFF]             shooterFault = true;
                // [OFF]             shooterOn = false;
                // [OFF]             commandedRpm = 0;
                // [OFF]         }
                // [OFF]     } else {
                // [OFF]         noFeedbackSince = -1;
                // [OFF]     }
                // [OFF]
                // [OFF]     double targetTps = commandedRpm * SHOOTER_TICKS_PER_REV / 60.0;
                // [OFF]     if (shooterOn && targetTps > 0) {
                // [OFF]         double error = targetTps - currentTps;
                // [OFF]         if (!shooterWasOn) shooterPreviousError = error;
                // [OFF]         if (Math.abs(error) < SHOOTER_INTEGRAL_ZONE) {
                // [OFF]             shooterTotalError += error * dt;
                // [OFF]         } else {
                // [OFF]             shooterTotalError = 0;
                // [OFF]         }
                // [OFF]         double derivative = dt > 0 ? (error - shooterPreviousError) / dt : 0;
                // [OFF]         shooterPreviousError = error;
                // [OFF]
                // [OFF]         double feedforward = SHOOTER_kS + SHOOTER_kV * targetTps;
                // [OFF]         double pid = SHOOTER_kP * error
                // [OFF]                 + SHOOTER_kI * shooterTotalError
                // [OFF]                 + SHOOTER_kD * derivative;
                // [OFF]         shooterPower = Range.clip(feedforward + pid, 0.0, 1.0);
                // [OFF]     } else {
                // [OFF]         shooterTotalError = 0;
                // [OFF]         shooterPreviousError = 0;
                // [OFF]     }
                // [OFF]     shooterWasOn = shooterOn;
                // [OFF]     shooter.setPower(shooterPower);
                // [OFF]
                // [OFF]     packet.put("Shooter target RPM", commandedRpm);
                // [OFF]     packet.put("Shooter actual RPM", actualRpm);
                // [OFF]     packet.put("Shooter power", shooterPower);
                // [OFF] }

                // ===================== SLIDE (gamepad 1 triggers) =====================
                double slidePosition = 0;
                if (slideEnabled) {
                    int leftPos  = leftSlide.getCurrentPosition();
                    int rightPos = rightSlide.getCurrentPosition();
                    slidePosition = (leftPos + rightPos) / 2.0;


                    // Hold LT = up (press harder = faster)
                    double up = gamepad1.left_trigger;     // 0.0 - 1.0
                    double slidePower ;
                    if (up > 0.5) {

                        slidePower=0.7;
                    }else{
                        slidePower =0;
                    }

                    // Hold RT = down (press harder = faster)
                    double down = gamepad1.right_trigger;  // 0.0 - 1.0
                    if (down > 0.5) {
                        slidePower =-0.7;
                    }
//
//                    // Release both = target stops changing, PID holds the slide there
//                    slideTarget = Range.clip(slideTarget, SLIDE_MIN, SLIDE_MAX);
//
//                    double error = slideTarget - slidePosition;
//                    slideTotalError += error * dt;
//                    double derivative = dt > 0 ? (error - slidePreviousError) / dt : 0;
//                    slidePreviousError = error;
//
//                    double slidePower = SLIDE_kP * error
//                            + SLIDE_kI * slideTotalError
//                            + SLIDE_kD * derivative
//                            + SLIDE_kG;
//
//                    // At the bottom: rest instead of fighting gravity
//                    if (slideTarget <= SLIDE_MIN + 10 && slidePosition < SLIDE_MIN + 20) {
//                        slidePower = 0;
//                        slideTotalError = 0;
//                    }
//                    slidePower = Range.clip(slidePower, -SLIDE_MAX_POWER, SLIDE_MAX_POWER);


                    leftSlide.setPower(Range.clip(slidePower, -1, 1));
                    rightSlide.setPower(Range.clip(slidePower, -1, 1));

                    packet.put("Slide target", slideTarget);
                    packet.put("Slide position", slidePosition);
                    packet.put("Slide left", leftPos);
                    packet.put("Slide right", rightPos);

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
                // [OFF] telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
                // [OFF] telemetry.addData("Shooter", shooterFault ? "FAULT: restart after checking encoder/stall"
                // [OFF]         : shooter == null ? "Not configured"
                // [OFF]         : !SHOOTER_ENCODER_CONFIRMED ? "Disabled: encoder not confirmed"
                // [OFF]         : shooterOn ? "ON" : "OFF");
                // [OFF] telemetry.addData("Motor target / measured RPM", "%.0f / %.0f", targetRpm, actualRpm);
                telemetry.addData("Slide", slideEnabled
                        ? String.format("target %.0f / pos %.0f", slideTarget, slidePosition)
                        : "Not configured (need motors 'ayc' and 'fiona')");
                telemetry.addData("LT / RT", "%.2f / %.2f", gamepad1.left_trigger, gamepad1.right_trigger);
                telemetry.addData("Hood", hoodEnabled ? "Level " + hoodLevel : "Disabled: not calibrated");
                telemetry.update();

                idle();
            }
        } finally {
            // [OFF] if (shooter != null) shooter.setPower(0);
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