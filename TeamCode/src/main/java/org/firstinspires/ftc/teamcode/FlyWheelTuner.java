package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class FlyWheelTuner extends OpMode {
    private DcMotorEx rightFlyWheel;

    public void init() {
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "turretturn");
        rightFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void loop(){
        if (gamepad1.right_trigger > 0.1) {
            rightFlyWheel.setPower(1);
        } else {
            rightFlyWheel.setPower(0);
        }
        telemetry.addData("Ramp Wheel velocity", "%.2f", rightFlyWheel.getVelocity());
        telemetry.update();
    }
}
