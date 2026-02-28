package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedNearAuto", group = "Auto")
public class RedNearAuto extends SimpleLineAutoBase {
    @Override
    protected Config getConfig() {
        // TODO: tune exact start/end coordinates for red near side.
        return config("Red Near Auto", 72, 24, 24, 5.0);
    }
}
