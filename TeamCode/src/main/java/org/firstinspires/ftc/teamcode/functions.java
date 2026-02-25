package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class functions {
    public static double getMotorExTPS(DcMotorEx motor) {
        return motor.getVelocity();
    }
    public static boolean isTPSLow (DcMotorEx motor, double expectedTPS){
        return (motor.getVelocity() < expectedTPS - 200);
    }
    public static void consecutiveShots(DcMotorEx flywheel1, DcMotorEx flywheel2, double expectedTPS){
        double curTPS1 = getMotorExTPS(flywheel1);
        double curTPS2 = getMotorExTPS(flywheel2);

    }
    public static void setupMotors(
            DcMotorEx leftFrontDrive,
            DcMotorEx rightFrontDrive,
            DcMotorEx leftBackDrive,
            DcMotorEx rightBackDrive,
            DcMotorEx leftFlyWheel,
            DcMotorEx rightFlyWheel,
            DcMotorEx doubleintake,
            DcMotorEx frontintake) {

        // chassis motors direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // chassis brake modes
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // chassis encoder setup
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // flywheel directions
        rightFlyWheel.setDirection(DcMotorEx.Direction.FORWARD);
        leftFlyWheel.setDirection(DcMotorEx.Direction.REVERSE);

        // flywheel encoder setup
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // flywheel PIDF setup
        PIDFCoefficients flywheelpidf = new PIDFCoefficients(3,0,0,7);
        leftFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelpidf);
        rightFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelpidf);

        // intake directions setup
        doubleintake.setDirection(DcMotorEx.Direction.FORWARD);
        frontintake.setDirection(DcMotorEx.Direction.FORWARD);

        // intake encoder setup
        doubleintake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontintake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // intake PIDF setup
        // TODO: TUNE PIDF FOR INTAKE
        PIDFCoefficients intakepidf = new PIDFCoefficients(0, 0, 0, 0);

        // PIDF intake setup
        doubleintake.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, intakepidf);
        frontintake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, intakepidf);


    }
}
