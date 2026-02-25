package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class DcMotorBasicTuner extends OpMode {
    private DcMotor motor;
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "frontintake");
    }
    public void loop(){
        if (gamepad1.right_trigger > 0.5){
            motor.setPower(1);
        } else {
            motor.setPower(0);
        }
    }
}
