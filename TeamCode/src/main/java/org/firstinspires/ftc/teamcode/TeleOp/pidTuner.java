package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Shooter PID Tuner")
public class pidTuner extends OpMode {
    // Editable live in FTC Dashboard
    public static double targetVelocity = 0; // ticks per second
    public static double kS = 0, kV = 0, kP = 0, kI = 0, kD = 0;

    public double currentVelocity;
    private DcMotorEx shooter;
    private ElapsedTime timer;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    private double previousError, totalError, errorDerivative, previousTime;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    // we do our own control
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // flywheel coasts

        timer = new ElapsedTime();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void start() {
        timer.reset();
        previousTime = 0;
        previousError = 0;
        totalError = 0;
    }

    @Override
    public void loop() {
        double currentTime = timer.seconds();
        double time_difference = currentTime - previousTime;

        if (time_difference > 0) {
            currentVelocity = shooter.getVelocity();
            double error = targetVelocity - currentVelocity;
            totalError += error * time_difference;
            errorDerivative = (error - previousError) / time_difference;

            double feedforward = kS * Math.signum(targetVelocity) + kV * targetVelocity;
            double pid = kP * error + kI * totalError + kD * errorDerivative;

            // 0 to 1 only: never drive the flywheel backwards
            double power = Math.max(0.0, Math.min(feedforward + pid, 1.0));

            if (targetVelocity == 0) {
                power = 0;
                totalError = 0;
            }
            shooter.setPower(power);

            previousError = error;
            previousTime = currentTime;

            packet = new TelemetryPacket();
            packet.put("Target Velocity", targetVelocity);
            packet.put("Current Velocity", currentVelocity);
            packet.put("Error", error);
            packet.put("Power", power);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Target", targetVelocity);
            telemetry.addData("Current", currentVelocity);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        shooter.setPower(0);
    }
}