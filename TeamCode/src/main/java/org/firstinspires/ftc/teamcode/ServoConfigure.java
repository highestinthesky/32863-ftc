package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoConfigure")
public class ServoConfigure extends OpMode {
    private Servo rightturretturn;
    private Servo leftturretturn;
    public double rightturretturnvalue;
    public double leftturretturnvalue;

    @Override
    public void init() {
        try {
            rightturretturn = hardwareMap.get(Servo.class, "rightturretturn");
            rightturretturn.setDirection(Servo.Direction.FORWARD);
        } catch (IllegalArgumentException ignored) {
            telemetry.addLine("rightturretturn not found in configuration");
        }

        leftturretturn = hardwareMap.get(Servo.class, "tservo");
        leftturretturn.setDirection(Servo.Direction.FORWARD);

        telemetry.addLine("init done");
    }

    @Override
    public void start() {
        if (rightturretturn != null) {
            rightturretturn.setPosition(0);
        }
        leftturretturn.setPosition(0);
    }

    @Override
    public void loop() {
        int action = 0;
        if (gamepad1.yWasPressed()) action = 3;
        else if (gamepad1.aWasPressed()) action = 4;

        switch (action) {
            case 3:
                if (leftturretturnvalue < 1) leftturretturnvalue += 0.1;
                break;
            case 4:
                if (leftturretturnvalue > 0) leftturretturnvalue -= 0.1;
        }

        if (rightturretturn != null) {
            rightturretturn.setPosition(rightturretturnvalue);
        }
        leftturretturn.setPosition(leftturretturnvalue);
        telemetry.update();
    }
}
