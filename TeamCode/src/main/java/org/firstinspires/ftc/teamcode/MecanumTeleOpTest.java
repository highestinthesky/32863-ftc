package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumTeleOpTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // declare basic motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        // reverse controls
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // factor to multiply sideways turning by
        double turn_scale = 0.5;

        waitForStart();
        // stop code, is necessary
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // this is fixed settings
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x * turn_scale;
            // denominator helps with equalizing power
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            // quick telemetry data so we're not blind + redundancy
            // currently not needed, enable later
            /*
            telemetry.addData("Front Left Motor Power:", frontLeftPower);
            telemetry.addData("Front Right Motor Power:", frontRightPower);
            telemetry.addData("Back Left Motor Power:", backLeftPower);
            telemetry.addData("Back Right Motor Power:", backRightPower);
            telemetry.update();
            */

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}
