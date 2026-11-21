package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Slide Tuner", group = "Tuner")
public class SlideTuner extends OpMode {


    public static double targetHeight = 0; // inches (ticks until TICKS_PER_INCH is set)

    private Slide slide;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        slide = new Slide(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry.addLine("Lift must be fully DOWN now.");
        telemetry.addLine("Gains: 'Slide' in Dashboard. Target: 'SlideTuner'.");
        telemetry.update();
    }

    @Override
    public void start() { slide.resetTimer(); }

    @Override
    public void loop() {
        slide.setTarget(targetHeight);
        slide.update();

        TelemetryPacket packet = new TelemetryPacket();
        slide.addTelemetry(packet);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Target", "%.2f", slide.getTarget());
        telemetry.addData("Position", "%.2f", slide.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() { slide.stop(); }
}