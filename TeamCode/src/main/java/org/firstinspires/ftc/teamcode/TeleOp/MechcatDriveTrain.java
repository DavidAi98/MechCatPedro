package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MechCat Drive", group = "MechCat")
public class MechcatDriveTrain extends LinearOpMode {

    private static final double INTAKE_POWER = 0.40;

    // Enable ONLY after connecting and checking the shooter encoder at school.
    private static final boolean SHOOTER_ENCODER_CONFIRMED = false;
    // Manufacturer sheet specifies 384.5 encoder counts per gearbox output turn.
    private static final double SHOOTER_TICKS_PER_REV = 384.5;
    private static final double SHOOTER_MAX_RPM = 435.0;
    // Initial PIDF values, NOT hardware-tuned. F uses nominal no-load velocity.
    private static final double SHOOTER_P = 10.0;
    private static final double SHOOTER_I = 0.0;
    private static final double SHOOTER_D = 0.0;
    private static final double SHOOTER_F =
            32767.0 / (SHOOTER_MAX_RPM * SHOOTER_TICKS_PER_REV / 60.0);

    // Never substitute arbitrary 0/1 endpoints. Calibrate with linkage detached.
    private static final boolean HOOD_CALIBRATED = false;
    private static final double HOOD_LEVEL_1 = Double.NaN;
    private static final double HOOD_LEVEL_5 = Double.NaN;

    @Override
    public void runOpMode() throws InterruptedException {
        // Configure the intake motor with the exact name "intake".
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        if (intake.getPortNumber() != 3) {
            throw new IllegalStateException("Configure intake on Expansion Hub motor port 3.");
        }
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        // Drive motors remain on ports 0-3 of one hub.
        DcMotor[] ports = new DcMotor[4];

        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            if (motor.getController() == intake.getController()) {
                continue;
            }
            int port = motor.getPortNumber();
            if (port < 0 || port >= ports.length || ports[port] != null) {
                throw new IllegalStateException("Expected four motors on one Control Hub.");
            }
            ports[port] = motor;
        }

        for (int port = 0; port < ports.length; port++) {
            if (ports[port] == null) {
                throw new IllegalStateException("Configure a motor on port " + port);
            }
        }

        for (DcMotor motor : ports) {
            if (motor.getController() != ports[0].getController()) {
                throw new IllegalStateException("All four drive motors must use the same Control Hub.");
            }
        }

        // Control Hub motor ports: 0 FL, 1 FR, 2 BL, 3 BR.
        DcMotor frontLeft  = ports[0];  // 左前
        DcMotor frontRight = ports[1];  // 右前
        DcMotor backLeft   = ports[2];  // 左后
        DcMotor backRight  = ports[3];  // 右后

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor[] allMotors = {frontLeft, frontRight, backLeft, backRight, intake};
        for (DcMotor motor : allMotors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Optional until wired/configured: missing shooter/hood does not block driving.
        DcMotorEx shooter = hardwareMap.tryGet(DcMotorEx.class, "shooter");
        Servo hood = hardwareMap.tryGet(Servo.class, "hood");
        if (shooter != null) {
            shooter.setPower(0);
            if (shooter.getController() != intake.getController()
                    || shooter.getPortNumber() != 0) {
                throw new IllegalStateException("Configure shooter on Expansion Hub motor port 0.");
            }
            shooter.setDirection(DcMotorSimple.Direction.FORWARD);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (SHOOTER_ENCODER_CONFIRMED) {
                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooter.setVelocityPIDFCoefficients(
                        SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
                shooter.setVelocity(0);
            }
        }
        boolean hoodEnabled = hood != null && HOOD_CALIBRATED
                && Double.isFinite(HOOD_LEVEL_1) && Double.isFinite(HOOD_LEVEL_5)
                && HOOD_LEVEL_1 >= 0 && HOOD_LEVEL_1 <= 1
                && HOOD_LEVEL_5 >= 0 && HOOD_LEVEL_5 <= 1;
        int hoodLevel = 1;
        // No hood position command at all while calibration is disabled.
        if (hoodEnabled) hood.setPosition(HOOD_LEVEL_1);
        telemetry.addLine("A: shooter toggle. D-pad left/right: motor target RPM.");
        telemetry.addLine("D-pad up/down: calibrated hood levels 1-5.");
        telemetry.addData("Shooter enabled", shooter != null && SHOOTER_ENCODER_CONFIRMED);
        telemetry.addData("Hood enabled", hoodEnabled);
        telemetry.addLine("MechCat Drive ready. Ports: 0 FL, 1 FR, 2 BL, 3 BR.");
        telemetry.addLine("Left stick: drive/strafe. Right stick: turn.");
        telemetry.addLine("Hold LB: slow. Release sticks to stop.");
        telemetry.addLine("Right bumper: toggle intake on/off (40% power).");
        telemetry.update();
        waitForStart();

        boolean intakeOn = false;
        // A button held during START must be released before toggling.
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
        ElapsedTime timer = new ElapsedTime();
        double previousTime = timer.seconds();

        try {
            while (opModeIsActive()) {
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

                boolean rightBumper = gamepad1.right_bumper;
                if (rightBumper && !previousRightBumper) {
                    intakeOn = !intakeOn;
                }
                previousRightBumper = rightBumper;
                intake.setPower(intakeOn ? INTAKE_POWER : 0.0);
                telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
                double now = timer.seconds();
                double dt = Range.clip(now - previousTime, 0.0, 0.1);
                previousTime = now;
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
                if (shooter != null && SHOOTER_ENCODER_CONFIRMED) {
                    double goal = shooterOn ? targetRpm : 0;
                    commandedRpm = shooterOn
                            ? Math.min(goal, commandedRpm + 200.0 * dt) : 0;
                    actualRpm = shooter.getVelocity() * 60.0 / SHOOTER_TICKS_PER_REV;
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
                    if (shooterOn) {
                        shooter.setVelocity(commandedRpm * SHOOTER_TICKS_PER_REV / 60.0);
                    } else {
                        shooter.setVelocity(0);
                        shooter.setPower(0);
                    }
                }
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
                telemetry.addData("Shooter", shooterFault ? "FAULT: restart after checking encoder/stall"
                        : shooter == null ? "Not configured"
                        : !SHOOTER_ENCODER_CONFIRMED ? "Disabled: encoder not confirmed"
                        : shooterOn ? "ON" : "OFF");
                telemetry.addData("Motor target / measured RPM", "%.0f / %.0f", targetRpm, actualRpm);
                telemetry.addData("Hood", hoodEnabled ? "Level " + hoodLevel : "Disabled: not calibrated");
                telemetry.update();

                idle();
            }
        } finally {
            if (shooter != null) {
                if (SHOOTER_ENCODER_CONFIRMED) shooter.setVelocity(0);
                shooter.setPower(0);
            }
            for (DcMotor motor : allMotors) {
                motor.setPower(0);
            }
        }
    }

    private double deadzone(double value) {
        return Math.abs(value) <= 0.05 ? 0.0 :
                Math.copySign((Math.abs(value) - 0.05) / 0.95, value);
    }
}