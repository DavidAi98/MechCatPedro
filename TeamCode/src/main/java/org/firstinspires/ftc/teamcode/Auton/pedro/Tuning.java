package org.firstinspires.ftc.teamcode.Auton.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.Auton.pedro.procedures.MecanumTuner;

public class Tuning {



    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }




}
