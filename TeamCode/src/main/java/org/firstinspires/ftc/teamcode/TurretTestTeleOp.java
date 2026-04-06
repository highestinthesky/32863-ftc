package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class TurretTestTeleOp extends OpMode {

    private DcMotorEx leftFlyWheel; 
    private DcMotorEx rightFlyWheel; 
    
    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightBackDrive;
    private FaultTolerantMecanumDrive driveController;
    private DcMotorEx leftIntake;
    private DcMotorEx rightIntake;

    private static final double P = 3.0;
    private static final double I = 0.0;
    private static final double D = 0.0;
    private static final double F = 7.0;

    private DcMotorEx rightFlyWheel;
    private DcMotorEx leftFlyWheel;
    private Servo turrethood;
    double curvelocity = 0;
    double turrethoodvalue = 0;
    double[] stepSizes = {1000.0, 500.0, 100.0, 10.0, 1.0};
    int stepIndex = 2;

    @Override
    public void init() {
        // chassis motors setup
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "lflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rflywheel");
        
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        rightIntake = hardwareMap.get(DcMotorEx.class, "rightintake");
        leftIntake = hardwareMap.get(DcMotorEx.class, "leftintake");

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
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveController = new FaultTolerantMecanumDrive(
                leftFrontDrive,
                rightFrontDrive,
                leftBackDrive,
                rightBackDrive,
                telemetry);

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "lflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rflywheel");
        // flywheel directions
        rightFlyWheel.setDirection(DcMotorEx.Direction.REVERSE);
        leftFlyWheel.setDirection( DcMotorEx.Direction.FORWARD);
        // flywheel encoder setup
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // flywheel PIDF setup
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        leftFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turrethood = hardwareMap.get(Servo.class, "tservo");
        turrethood.setDirection(Servo.Direction.REVERSE);
        turrethood.setPosition(turrethoodvalue);
    }

    @Override
    public void start() {
        if (driveController != null) driveController.resetForStart();
    }

    @Override
    public void loop() {
        if gamepad2.bWasPressed() {
            rightFlyWheel.setPower(1);
            leftFlyWheel.setPower(1);
        }
    }

    @Override
    public void stop() {
        if (driveController != null) driveController.stopSafely();
        FaultTolerantMecanumDrive.MotorSafety.setPower(leftIntake, 0);
        FaultTolerantMecanumDrive.MotorSafety.setPower(rightIntake, 0);
        FaultTolerantMecanumDrive.MotorSafety.setVelocity(leftFlyWheel, 0);
        FaultTolerantMecanumDrive.MotorSafety.setVelocity(rightFlyWheel, 0);
    }

}
