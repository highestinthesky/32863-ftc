package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueNearAuto", group = "Auto")
public class BlueNearAuto extends SimpleLineAutoBase {
    @Override
    protected Config getConfig() {
        // TODO: tune exact start/end coordinates for blue near side.
        return config("Blue Near Auto", 72, 120, 24, 5.0);
    }
}
