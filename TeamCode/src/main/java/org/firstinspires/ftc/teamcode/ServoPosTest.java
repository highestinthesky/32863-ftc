package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoPosTest", group="Test")
public class ServoPosTest extends LinearOpMode {
    @Override public void runOpMode() {
        Servo left = hardwareMap.get(Servo.class, "lspindexerup");
        Servo right = hardwareMap.get(Servo.class, "rspindexerup");

        left.scaleRange(0.0, 1.0);
        right.scaleRange(0.0, 1.0);

        left.setPosition(0.2);
        right.setPosition(0.2);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                left.setPosition(0.8);
                right.setPosition(0.8);
            } else if (gamepad1.a) {
                left.setPosition(0.2);
                right.setPosition(0.2);
            }

            telemetry.addData("Y pressed", gamepad1.y);
            telemetry.addData("A pressed", gamepad1.a);
            telemetry.addData("L pos", left.getPosition());
            telemetry.addData("R pos", right.getPosition());
            telemetry.update();
        }
    }
}
