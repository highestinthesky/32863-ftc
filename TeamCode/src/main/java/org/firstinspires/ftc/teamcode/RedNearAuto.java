package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedNearAuto", group = "Auto")
public class RedNearAuto extends VisionFeedNearAutoBase {
    @Override
    protected Config getConfig() {
        return config(
                "Red Near Auto",
                72,    // startX
                24,    // startY
                30.0,  // move backward inches
                5.0,   // path timeout
                24,    // red goal tag id
                1,     // red AprilTag pipeline
                2.0,   // tag acquire timeout
                2.0,   // flywheel spinup timeout
                1.0    // intake feed duration
        );
    }
}
