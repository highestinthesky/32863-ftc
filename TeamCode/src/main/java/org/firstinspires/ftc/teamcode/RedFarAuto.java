package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedFarAuto", group = "Auto")
public class RedFarAuto extends SimpleLineAutoBase {
    @Override
    protected Config getConfig() {
        // TODO: tune exact start/end coordinates for red far side.
        return config("Red Far Auto", 72, 48, 20, 5.0);
    }
}
