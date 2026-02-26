package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.lib.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="MecanumTeleOpRunVer")
public class MecanumTeleOpRunVer extends OpMode {


    // Drive motors
    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightBackDrive;
    private FaultTolerantMecanumDrive driveController;
    // Other motors/servos
    private DcMotorEx doubleintake;
    private DcMotorEx leftFlyWheel;
    private DcMotorEx rightFlyWheel;
    private DcMotorEx frontintake;

    private Servo lspindexerup;
    private Servo rspindexerup;
    private double tservoPosition = 0.0;

    public double highVelocity = 6000;
    public double lowVelocity = 1500;
    double curTargetVelocity = highVelocity;
    public double basespindexerup = 180;
    public double basespindexerdown = 70;
    double curTargetspindexer = basespindexerdown;
    double rampwheelOn = 1150;
    double rampwheelOff = 0;
    double rampwheelbackwards = -300;

    private boolean stopping = false;

    private ElapsedTime indexerAndRampWheelTimer = new ElapsedTime();
    private boolean indexerAndRampWheelTimerActive = false;
    private boolean overrideMode = false;

    // Pinpoint placeholder
    private PinpointIO pinpoint;

    @Override
    public void init() {
        // chassis motors setup
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // intake setup call
        doubleintake = hardwareMap.get(DcMotorEx.class, "doubleintake");
        frontintake = hardwareMap.get(DcMotorEx.class, "frontintake");

        // flywheel setup call
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "lflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rflywheel");

        functions.setupMotors(
                leftFrontDrive,
                rightFrontDrive,
                leftBackDrive,
                rightBackDrive,
                leftFlyWheel,
                rightFlyWheel,
                doubleintake,
                frontintake);

        driveController = new FaultTolerantMecanumDrive(
                leftFrontDrive,
                rightFrontDrive,
                leftBackDrive,
                rightBackDrive,
                telemetry);

        // tservo setup call
        Servo tservo = hardwareMap.get(Servo.class, "tservo");
        // tservo directions
        tservo.setDirection(Servo.Direction.REVERSE);

        //spindexerup setup calls
        lspindexerup = hardwareMap.get(Servo.class, "lspindexerup");
        rspindexerup = hardwareMap.get(Servo.class, "rspindexerup");
        // spindexerup directions
        lspindexerup.setDirection(Servo.Direction.FORWARD);
        rspindexerup.setDirection(Servo.Direction.FORWARD);


        // ---- Pinpoint (placeholder) ----
        pinpoint = new PinpointIO(hardwareMap, telemetry, "odo");
        pinpoint.initializeAndConfigure();

        telemetry.addLine("Initialized. Press PLAY.");
        telemetry.update();
    }

    @Override
    public void start() {
        // Set initial servo positions to 70 degrees
        functions.setServoAngle(lspindexerup, 70);
        functions.setServoAngle(rspindexerup, 70);
        if (driveController != null) driveController.resetForStart();
    }

    @Override
    public void loop() {
        if (stopping) return;
        pinpoint.update();
//        int action = 0;
//        if (gamepad1.bWasPressed()) action = 2;
//        else if (gamepad1.dpadLeftWasPressed()) action = 3;
//        else if (gamepad1.dpadRightWasPressed()) action = 4;
//        else if (gamepad1.dpadUpWasPressed()) action = 5;
//        else if (gamepad1.dpadDownWasPressed()) action = 6;
//
//        switch (action) {
//            case 2:
//                stepIndex = (stepIndex + 1) % stepSizes.length;
//                break;
//            case 3:
//                F -= stepSizes[stepIndex];
//                break;
//            case 4:
//                F += stepSizes[stepIndex];
//                break;
//            case 5:
//                P += stepSizes[stepIndex];
//                break;
//            case 6:
//                P -= stepSizes[stepIndex];
//                break;
//        }
        //Flywheel controlling velocity
        if (gamepad1.yWasPressed()) curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;

        // activating overrideMode
        if (gamepad2.aWasPressed() && !gamepad2.aWasReleased()) {overrideMode = true;}
        else {overrideMode = false;}

        // code to change rampwheel to go backwards
        if (overrideMode && gamepad2.right_trigger > 0.1) {
            doubleintake.setVelocity(rampwheelbackwards);
        } else if (overrideMode){
            doubleintake.setVelocity(rampwheelOff);
        }

        // --- Indexer control + rampWheel control (gamepad2 B) ---
        if (gamepad2.bWasPressed()) {
            PIDFCoefficients rampWheelCoefficients = new PIDFCoefficients(4,0,0,20);
            doubleintake.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, rampWheelCoefficients);
            doubleintake.setVelocity(rampwheelOn);
            curTargetspindexer = basespindexerup;
            indexerAndRampWheelTimer.reset();
            indexerAndRampWheelTimerActive = true;
        }
        if (indexerAndRampWheelTimerActive && indexerAndRampWheelTimer.seconds() >= 0.3 && indexerAndRampWheelTimer.seconds() <= 0.6) {
            curTargetspindexer = basespindexerdown;
        }
        if (indexerAndRampWheelTimerActive && indexerAndRampWheelTimer.seconds() >= 0.8) {
            doubleintake.setVelocity(rampwheelOff);
            indexerAndRampWheelTimerActive = false;
        }
        functions.setServoAngle(lspindexerup, curTargetspindexer);
        functions.setServoAngle(rspindexerup, curTargetspindexer);


//         --- Turret rotation (gamepad2 bumpers) ---
//        int turretState = 0;
//        if (gamepad2.right_bumper) turretState = 1;
//        else if (gamepad2.left_bumper) turretState = 2;
//
//        switch (turretState) {
//            case 1:
//                turretRotation.setPower(1);
//                break;
//            case 2:
//                turretRotation.setPower(-1);
//                break;
//            default:
//                turretRotation.setPower(0);
//                break;
//        }

        // --- T-Servo incremental control (gamepad2 X/Y) ---
//        if (gamepad2.x) {
//            tservoPosition += 0.1;
//        } else if (gamepad2.y) {
//            tservoPosition -= 0.1;
//        }

//        if (tservoPosition > 1.0) tservoPosition = 1.0;
//        else if (tservoPosition < 0.0) tservoPosition = 0.0;
//
//        tservo.setPosition(tservoPosition);

        // --- Intake (gamepad2 left trigger) ---
        if (gamepad2.left_trigger > 0.1) {
            if (overrideMode) {
                doubleintake.setPower(-1);
            } else {
                doubleintake.setPower(1);
            }
        } else {
            doubleintake.setPower(0);
        }
        
        if (gamepad2.right_bumper) {
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(12,0,0,4);
            leftFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            rightFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            leftFlyWheel.setVelocity(curTargetVelocity);
            rightFlyWheel.setVelocity(curTargetVelocity);
        } else{
            leftFlyWheel.setVelocity(0);
            rightFlyWheel.setVelocity(0);
        }

        // --- Drive control ---
        double drive  = -gamepad1.left_stick_y;        
        double strafe = -gamepad1.left_stick_x;        
        double turn   = -gamepad1.right_stick_x;

        if (driveController != null) driveController.move(drive, strafe, turn);
        

        telemetry.addLine("--------------------------------------");
        telemetry.addData("T-Servo Position", "%.3f", tservoPosition);
        telemetry.addData("Override Mode", overrideMode);
        telemetry.addData("Odo X (in)", "%.1f", pinpoint.getXInches());
        telemetry.addData("Odo Y (in)", "%.1f", pinpoint.getYInches());
        telemetry.addData("Odo H (deg)", "%.1f", Math.toDegrees(pinpoint.getHeadingRadians()));
        telemetry.update();
    }

    @Override
    public void stop() {
        stopping = true;

        // Stop velocity-controlled motors first
        FaultTolerantMecanumDrive.MotorSafety.setVelocity(leftFlyWheel, 0);
        FaultTolerantMecanumDrive.MotorSafety.setVelocity(rightFlyWheel, 0);
        FaultTolerantMecanumDrive.MotorSafety.setVelocity(doubleintake, 0);

        // Stop everything else
        if (driveController != null) driveController.stopSafely();
        FaultTolerantMecanumDrive.MotorSafety.setPower(doubleintake, 0);

    }
//    public void moveRobot(double drive, double strafe, double turn) {
//        double leftFrontPower  = drive - strafe - turn;
//        double rightFrontPower = drive + strafe + turn;
//        double leftBackPower   = drive + strafe - turn;
//        double rightBackPower  = drive - strafe + turn;
//
//        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > prevMax + 0.075) {
//            max = prevMax + 0.075;
//            prevMax = max;
//
//            leftFrontPower  *= max;
//            rightFrontPower *= max;
//            leftBackPower   *= max;
//            rightBackPower  *= max;
//        } else if (max > 1.0) {
//            leftFrontPower  /= max;
//            rightFrontPower /= max;
//            leftBackPower   /= max;
//            rightBackPower  /= max;
//        } else {
//            prevMax = max;
//        }
//
//        leftFrontDrive.setPower(leftFrontPower);
//        rightFrontDrive.setPower(rightFrontPower);
//        leftBackDrive.setPower(leftBackPower);
//        rightBackDrive.setPower(rightBackPower);
//    }

    private static class PinpointIO {

        private final GoBildaPinpointDriver odo;
        private final Telemetry telemetry;

        PinpointIO(HardwareMap hardwareMap, Telemetry telemetry, String deviceName) {
            this.telemetry = telemetry;
            this.odo = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);
        }

        void initializeAndConfigure() {
            // MUST be done at init/start while robot is still
            odo.resetPosAndIMU();
            telemetry.addLine("Pinpoint initialized");
            telemetry.update();
        }

        void update() {
            odo.update();
        }
        double getXInches() {
            return odo.getPosition().getX(DistanceUnit.INCH);
        }
        double getYInches() {
            return odo.getPosition().getY(DistanceUnit.INCH);
        }
        double getHeadingRadians() {
            return odo.getPosition().getHeading(AngleUnit.RADIANS);
        }
    }
}
