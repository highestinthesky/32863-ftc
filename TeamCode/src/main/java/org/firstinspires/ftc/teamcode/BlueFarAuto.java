package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "BlueFarAuto", group = "Auto")
public class BlueFarAuto extends LinearOpMode {
    private static final double DRIVE_POWER = 0.4;
    private static final long DRIVE_TIME_MS = 1000;

    @Override
    public void runOpMode() {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("BlueFarAuto ready: raw forward 1s");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        leftFront.setPower(DRIVE_POWER);
        rightFront.setPower(DRIVE_POWER);
        leftBack.setPower(DRIVE_POWER);
        rightBack.setPower(DRIVE_POWER);

        sleep(DRIVE_TIME_MS);

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);
    }
}
