package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoConfigure")
public class ServoConfigure extends OpMode {
    private Servo rightturretturn;
    private Servo leftturretturn;
    public double turretturnvalue;

    @Override
    public void init() {

        rightturretturn = hardwareMap.get(Servo.class, "rightturretturn");
        leftturretturn = hardwareMap.get(Servo.class, "leftturretturn");

        leftturretturn.setDirection(Servo.Direction.FORWARD);
        rightturretturn.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("init done");
    }

    @Override
    public void start() {
        rightturretturn.setPosition(0);
        leftturretturn.setPosition(0);
    }

    @Override
    public void loop() {
        int action = 0;
        if (gamepad1.yWasPressed()) action = 3;
        else if (gamepad1.aWasPressed()) action = 4;

        switch (action) {
            case 3:
                if (turretturnvalue < 1) turretturnvalue += 0.1;
                break;
            case 4:
                if (turretturnvalue > 0) turretturnvalue -= 0.1;
        }

        rightturretturn.setPosition(turretturnvalue);
        leftturretturn.setPosition(turretturnvalue);
        telemetry.update();
    }
}
