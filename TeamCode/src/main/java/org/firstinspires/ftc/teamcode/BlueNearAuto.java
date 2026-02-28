package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueNearAuto", group = "Auto")
public class BlueNearAuto extends VisionFeedNearAutoBase {
    @Override
    protected Config getConfig() {
        return config(
                "Blue Near Auto",
                72,    // startX
                120,   // startY
                30.0,  // move backward inches
                5.0,   // path timeout
                20,    // blue goal tag id
                0,     // blue AprilTag pipeline
                2.0,   // tag acquire timeout
                2.0,   // flywheel spinup timeout
                1.0    // intake feed duration
        );
    }
}
