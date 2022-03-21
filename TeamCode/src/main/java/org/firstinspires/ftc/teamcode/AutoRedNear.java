package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="autorednear")
public class AutoRedNear extends GeneralAuto {
    @Override
    public void movement(int level) {
        telemetry.addData("It returned", level);
    }

    @Override
    public int recognition() {
        return 0;
    }
}