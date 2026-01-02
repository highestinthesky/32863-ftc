package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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
    private Servo lspindexerup;
    private Servo rspindexerup;
    private double tservoPosition = 0.0;

    // Optional ramp limiter state
    private static double prevMax = 0.275;
    public double highVelocity = 6000;
    public double lowVelocity = 1500;
    double curTargetVelocity = highVelocity;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.001};
    int stepIndex = 1;
    public double basespindexerup = 180;
    public double basespindexerdown = 70;
    double curTargetspindexer = basespindexerdown;

    // Timer for temporary indexer movement
    private ElapsedTime indexerTimer = new ElapsedTime();
    private boolean indexerTimerActive = false;

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
        lspindexerup = hardwareMap.get(Servo.class, "lspindexerup");
        rspindexerup = hardwareMap.get(Servo.class, "rspindexerup");

        // ---- Directions ----
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        rightFlyWheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Mirror the indexer servos so they move together
        lspindexerup.setDirection(Servo.Direction.FORWARD);
        rspindexerup.setDirection(Servo.Direction.FORWARD);
        tservo.setDirection(Servo.Direction.REVERSE);

        // Set initial servo positions to 70 degrees
        setServoAngle(lspindexerup, 70);
        setServoAngle(rspindexerup, 70);

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
        int action = 0;
        if (gamepad1.yWasPressed()) action = 1;
        else if (gamepad1.bWasPressed()) action = 2;
        else if (gamepad1.dpadLeftWasPressed()) action = 3;
        else if (gamepad1.dpadRightWasPressed()) action = 4;
        else if (gamepad1.dpadUpWasPressed()) action = 5;
        else if (gamepad1.dpadDownWasPressed()) action = 6;

        switch (action) {
            case 1:
                curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
                break;
            case 2:
                stepIndex = (stepIndex + 1) % stepSizes.length;
                break;
            case 3:
                F -= stepSizes[stepIndex];
                break;
            case 4:
                F += stepSizes[stepIndex];
                break;
            case 5:
                P += stepSizes[stepIndex];
                break;
            case 6:
                P -= stepSizes[stepIndex];
                break;
        }

        // --- Spindexer (gamepad1 triggers) ---
        int spindexerState = 0;
        if (gamepad1.right_trigger > 0.1) spindexerState = 1;
        else if (gamepad1.left_trigger > 0.1) spindexerState = 2;
        
        switch (spindexerState) {
            case 1:
                spindexer.setPower(1);
                break;
            case 2:
                spindexer.setPower(-1);
                break;
            default:
                spindexer.setPower(0);
                break;
        }
        
        // --- Indexer control (gamepad2 B) ---
        if (gamepad2.bWasPressed()) {
            curTargetspindexer = basespindexerup;
            indexerTimer.reset();
            indexerTimerActive = true;
        }

        if (indexerTimerActive && indexerTimer.seconds() >= 0.6) {
            curTargetspindexer = basespindexerdown;
            indexerTimerActive = false;
        }

        setServoAngle(lspindexerup, curTargetspindexer);
        setServoAngle(rspindexerup, curTargetspindexer);

        // --- Turret rotation (gamepad2 bumpers) ---
        int turretState = 0;
        if (gamepad2.right_bumper) turretState = 1;
        else if (gamepad2.left_bumper) turretState = 2;

        switch (turretState) {
            case 1:
                turretRotation.setPower(1);
                break;
            case 2:
                turretRotation.setPower(-1);
                break;
            default:
                turretRotation.setPower(0);
                break;
        }

        // --- T-Servo incremental control (gamepad2 X/Y) ---
        if (gamepad2.x) {
            tservoPosition += 0.1;
        } else if (gamepad2.y) {
            tservoPosition -= 0.1;
        }

        if (tservoPosition > 1.0) tservoPosition = 1.0;
        else if (tservoPosition < 0.0) tservoPosition = 0.0;
        
        tservo.setPosition(tservoPosition);

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
        double drive  = -gamepad1.left_stick_y;        
        double strafe = -gamepad1.left_stick_x;        
        double turn   = -gamepad1.right_stick_x / 2.0; 

        moveRobot(drive, strafe, turn);
        
        telemetry.addData("Target velocity", curTargetVelocity);
        telemetry.addData("Current velocity", "%.2f", averagecurvelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("--------------------------------------");
        telemetry.addData("T-Servo Position", "%.3f", tservoPosition);
        telemetry.addData("Indexer L Angle", "%.1f", lspindexerup.getPosition() * 300.0);
        telemetry.addData("Indexer R Angle", "%.1f", rspindexerup.getPosition() * 300.0);
        telemetry.addData("P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
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
    }

    /**
     * Sets a servo position based on degrees (0-300 for "full range" servos).
     */
    public void setServoAngle(Servo servo, double angle) {
        servo.setPosition(angle / 300.0);
    }

    public void moveRobot(double drive, double strafe, double turn) {
        double leftFrontPower  = drive - strafe - turn;
        double rightFrontPower = drive + strafe + turn;
        double leftBackPower   = drive + strafe - turn;
        double rightBackPower  = drive - strafe + turn;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

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
