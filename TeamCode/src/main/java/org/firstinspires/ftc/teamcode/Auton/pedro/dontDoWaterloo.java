package org.firstinspires.ftc.teamcode.Auton.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Leave Zone (20 in)", group = "Auton")
public class dontDoWaterloo extends LinearOpMode {

    // ---- Settings ----
    private static final double DISTANCE_INCHES = 20;
    private static final double DRIVE_POWER = 0.4;
    private static final double TIMEOUT_SECONDS = 5;

    // ---- Drivetrain measurements: CHECK THESE FOR YOUR ROBOT ----
    // goBILDA 312 RPM motor = 537.7 ticks/rev, 435 RPM = 384.5, 1150 RPM = 145.1
    private static final double TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER_INCHES = 104 / 25.4; // goBILDA 104mm mecanum
    private static final double GEAR_RATIO = 1.0; // wheel turns per motor turn
    private static final double TICKS_PER_INCH =
            (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() {
        // Same names and directions as MechCat Drive
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight};
        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        telemetry.addData("Ready", "Drive forward %.0f in", DISTANCE_INCHES);
        telemetry.addData("Ticks per inch", "%.1f", TICKS_PER_INCH);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---- Drive forward ----
        int target = (int) Math.round(DISTANCE_INCHES * TICKS_PER_INCH);
        for (DcMotor m : motors) {
            m.setTargetPosition(target);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(DRIVE_POWER);
        }

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()
                && timer.seconds() < TIMEOUT_SECONDS
                && (frontLeft.isBusy() || frontRight.isBusy()
                || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("Target", target);
            telemetry.addData("FL / FR", "%d / %d",
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.addData("BL / BR", "%d / %d",
                    backLeft.getCurrentPosition(), backRight.getCurrentPosition());
            telemetry.update();
        }

        // ---- Stop ----
        for (DcMotor m : motors) m.setPower(0);

        telemetry.addLine(timer.seconds() >= TIMEOUT_SECONDS
                ? "Stopped by timeout: check encoders!" : "Done ✓");
        telemetry.update();
        while (opModeIsActive()) idle(); // wait for auton to end
    }
}