package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoConfigure extends OpMode {
    private Servo rightturretturn;
    private Servo leftturretturn;
    public double rightturretturnvalue;
    public double leftturretturnvalue;
    @Override
    public void init() {
        rightturretturn = hardwareMap.get(Servo.class, "rightturretturn");
        leftturretturn = hardwareMap.get(Servo.class, "leftturretturn");
        rightturretturn.setDirection(Servo.Direction.FORWARD);
        leftturretturn.setDirection(Servo.Direction.FORWARD);

        telemetry.addLine("init done");
    }

    @Override
    public void start(){
        rightturretturn.setPosition(0);
        leftturretturn.setPosition(0);
    }
    @Override
    public void loop() {
        telemetry.update();
        int action = 0;
        if (gamepad1.dpadUpWasPressed()) action = 1;
        else if (gamepad1.dpadDownWasPressed()) action = 2;
        else if (gamepad1.yWasPressed()) action  = 3;
        else if (gamepad1.aWasPressed()) action  = 4;

        switch (action) {
            case 1:
                if (rightturretturnvalue < 1) rightturretturnvalue += 0.1;
                break;
            case 2:
                if (rightturretturnvalue > 0) rightturretturnvalue -= 0.1;
                break;
            case 3:
                if (leftturretturnvalue < 1) leftturretturnvalue += 0.1;
                break;
            case 4:
                if (leftturretturnvalue > 0) leftturretturnvalue -= 0.1;
        }

        rightturretturn.setPosition(rightturretturnvalue);
        leftturretturn.setPosition(leftturretturnvalue);
    }
}
