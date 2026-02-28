package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueFarAuto", group = "Auto")
public class BlueFarAuto extends SimpleLineAutoBase {
    @Override
    protected Config getConfig() {
        // TODO: tune exact start/end coordinates for blue far side.
        return config("Blue Far Auto", 72, 96, 24, 5.0);
    }
}
