package org.firstinspires.ftc.teamcode.TeleOp.hj;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;
@Config

@TeleOp()
public class pid_tuner extends OpMode {

    public double currentVelocity;
    public static double targetVelocity = 0, kP = 0, kI = 0, kD = 0, kS = 0.2717676, kV = 0.0002855;
    private double previousError = 0, total_error = 0, error_derivative = 0, previousTime = 0;
    private ElapsedTime timer;
    public DcMotorEx shooter;
    public FtcDashboard dashboard;
    public TelemetryPacket packet;
    public void init(){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        timer = new ElapsedTime();
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }
    public void loop(){
        double currentTime = timer.time(TimeUnit.MILLISECONDS);
        double time_difference = currentTime - previousTime;
        if (time_difference > 0){
            currentVelocity = shooter.getVelocity();
            double error = targetVelocity - currentVelocity;
            total_error += error * time_difference;
            error_derivative = (error - previousError) / time_difference;

            double feedforward = kS + kV * targetVelocity;
            double pid = kP * error + kI * total_error + kD * error_derivative;

            double power = feedforward + pid;

            shooter.setPower(Math.max(-1.0, Math.min(power, 1.0)));

            previousError = error;
            previousTime = currentTime;

            telemetry.addData("PID calculated", pid);
            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Current Position", shooter.getCurrentPosition());
            telemetry.addData("kS", kS);
            telemetry.addData("kV", kV);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.update();
            packet.put("Target Velocity", targetVelocity);
            packet.put("Current Velocity", currentVelocity);
            packet.put("kS", kS);
            packet.put("kV", kV);
            packet.put("kP", kP);
            packet.put("kI", kI);
            packet.put("kD", kD);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
