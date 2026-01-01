package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="MecanumTeleOpRunVer", group="Test")
public class MecanumTeleOpRunVer extends OpMode {

    // Drive motors
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    // Other motors/servos
    private DcMotor intake;
    private DcMotorEx leftFlyWheel;
    private DcMotorEx rightFlyWheel;
    private DcMotor turretRotation;

    private CRServo spindexer;
    private Servo tservo;
    private CRServo lspindexerup;
    private CRServo rspindexerup;

    // Optional ramp limiter state
    private static double prevMax = 0.275;
    public double highVelocity = 6000;
    public double lowVelocity = 1500;
    double curTargetVelocity = highVelocity;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.001};
    int stepIndex = 1;

    // Pinpoint placeholder
    private PinpointIO pinpoint;

    @Override
    public void init() {
        // ---- Motors ----
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "backRightMotor");

        intake = hardwareMap.get(DcMotor.class, "intake");
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "lflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rflywheel");
        turretRotation = hardwareMap.get(DcMotor.class, "turretturn"); // need to configure

        spindexer = hardwareMap.get(CRServo.class, "spindexer");
        tservo = hardwareMap.get(Servo.class, "tservo");
        lspindexerup = hardwareMap.get(CRServo.class, "lspindexerup");
        rspindexerup = hardwareMap.get(CRServo.class, "rspindexerup");

        // ---- Directions ----
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        rightFlyWheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        rspindexerup.setDirection(CRServo.Direction.REVERSE);
        tservo.setDirection(Servo.Direction.REVERSE);

        // ---- Run modes ----
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        leftFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        turretRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---- Pinpoint (placeholder) ----
        pinpoint = new PinpointIO(hardwareMap, telemetry, "odo");
        pinpoint.initializeAndConfigure();

        telemetry.addLine("Initialized. Press PLAY.");
        telemetry.update();
    }

    @Override
    public void start() {
        // Called once when you press PLAY
        prevMax = 0.275; // reset ramp at start too
    }

    @Override
    public void loop() {

        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {curTargetVelocity = highVelocity;}
        }
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        } if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        } if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        } if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        // --- Spindexer up servos (gamepad1 left trigger) ---
        if (gamepad1.left_trigger > 0.1) {
            lspindexerup.setPower(1);
            rspindexerup.setPower(1);
        } else {
            lspindexerup.setPower(0);
            rspindexerup.setPower(0);
        }

        // --- Spindexer (gamepad1 right trigger) ---
        if (gamepad1.right_trigger > 0.1) {
            spindexer.setPower(1);
        } else {
            spindexer.setPower(0);
        }
        // --- Tservo positions (gamepad1 y/x) ---
//        if (gamepad1.aWasPressed()) {
//            tservo.setPosition(0.5);
//        } else if (gamepad1.xWasPressed()) {
//            tservo.setPosition(0);
//        }x
        // --- Turret rotation (gamepad2 bumpers) ---
        if (gamepad2.right_bumper) {
            turretRotation.setPower(1);
        } else if (gamepad2.left_bumper) {
            turretRotation.setPower(-1);
        } else {
            turretRotation.setPower(0);
        }

//        // --- Flywheel (gamepad2 right trigger) ---
//        if (gamepad2.right_trigger > 0.1) {
//            leftFlyWheel.setPower(1);
//            rightFlyWheel.setPower(1);
//        } else {
//            leftFlyWheel.setPower(0);
//            rightFlyWheel.setPower(0);
//        }

        // --- Intake (gamepad2 left trigger) ---
        if (gamepad2.left_trigger > 0.1) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        leftFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftFlyWheel.setVelocity(curTargetVelocity);
        rightFlyWheel.setVelocity(curTargetVelocity);
        double leftcurVelocity = leftFlyWheel.getVelocity();
        double rightcurVelocity = rightFlyWheel.getVelocity();
        double averagecurvelocity = ((leftcurVelocity + rightcurVelocity)/2);
        double error = curTargetVelocity - averagecurvelocity;


        // --- Drive control ---
        double drive  = -gamepad1.left_stick_y;        // forward = +
        double strafe = -gamepad1.left_stick_x;        // right/left
        double turn   = -gamepad1.right_stick_x / 2.0; // slow turning

        moveRobot(drive, strafe, turn);
        telemetry.addData("Target velocity", curTargetVelocity);
        telemetry.addData("Current velocity", "%.2f", averagecurvelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.update();

    }

    @Override
    public void stop() {
        // Optional: ensure everything is off when opmode stops
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        intake.setPower(0);
        leftFlyWheel.setPower(0);
        rightFlyWheel.setPower(0);
        turretRotation.setPower(0);

        spindexer.setPower(0);
        lspindexerup.setPower(0);
        rspindexerup.setPower(0);
    }

    public void moveRobot(double drive, double strafe, double turn) {
        double leftFrontPower  = drive - strafe - turn;
        double rightFrontPower = drive + strafe + turn;
        double leftBackPower   = drive + strafe - turn;
        double rightBackPower  = drive - strafe + turn;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        // NOTE: This "ramp limiter" logic scales ALL wheel powers by "max"
        // only when max increases quickly. This is unusual; leaving it as-is
        // to preserve your behavior.
        if (max > prevMax + 0.075) {
            max = prevMax + 0.075;
            prevMax = max;

            leftFrontPower  *= max;
            rightFrontPower *= max;
            leftBackPower   *= max;
            rightBackPower  *= max;
        } else if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        } else {
            prevMax = max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Minimal placeholder
     */
    private static class PinpointIO {
        private final Telemetry telemetry;
        PinpointIO(HardwareMap hardwareMap, Telemetry telemetry, String deviceName) {
            this.telemetry = telemetry;
        }
        void initializeAndConfigure() {
            telemetry.addLine("Pinpoint: placeholder (no mock pose)");
            telemetry.update();
        }
    }
}
