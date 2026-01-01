package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="ServoTest", group="Test")
public class ServoTest extends LinearOpMode {
    @Override public void runOpMode() {
        CRServo l = hardwareMap.get(CRServo.class, "lspindexerup");
        CRServo r = hardwareMap.get(CRServo.class, "rspindexerup");

        waitForStart();
        while (opModeIsActive()) {
            double p = gamepad1.a ? 1.0 : (gamepad1.b ? -1.0 : 0.0);
            l.setPower(p);
            r.setPower(p);

            telemetry.addData("A=+1 B=-1 power", p);
            telemetry.update();
            idle();
        }
    }
}
