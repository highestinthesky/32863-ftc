package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shooter.ShooterData;

@TeleOp(name = "MecanumTeleOpRunVer")
public class MecanumTeleOpRunVer extends OpMode {
    private static final double DEFAULT_FLYWHEEL_TARGET_VELOCITY = 6000.0;
    private static final double INDEXER_UP_DEGREES = 180.0;
    private static final double INDEXER_DOWN_DEGREES = 70.0;
    private static final double RAMP_WHEEL_FORWARD_VELOCITY = 1150.0;
    private static final double RAMP_WHEEL_REVERSE_VELOCITY = -300.0;
    private static final int LIMELIGHT_APRILTAG_PIPELINE = 21;

    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightBackDrive;
    private FaultTolerantMecanumDrive driveController;

    private DcMotorEx rightintake;
    private DcMotorEx leftintake;
    private DcMotorEx leftFlyWheel;
    private DcMotorEx rightFlyWheel;

    private Servo leftSpindexerUp;
    private Servo rightSpindexerUp;

    public double manualFlywheelTargetVelocity = DEFAULT_FLYWHEEL_TARGET_VELOCITY;
    public boolean useVisionFlywheelTarget = false;

    private double activeFlywheelTargetVelocity = DEFAULT_FLYWHEEL_TARGET_VELOCITY;
    private boolean stopping = false;
    private boolean overrideMode = false;

    private Double limelightShooterDistanceInches = null;

    private final ShooterData shooterData = ShooterData.defaultTuned();
    private final functions.TimedShotSequence shotSequence = new functions.TimedShotSequence(
            INDEXER_UP_DEGREES,
            INDEXER_DOWN_DEGREES,
            RAMP_WHEEL_FORWARD_VELOCITY,
            0.3,
            0.8
    );

    private functions.PinpointIO pinpoint;
    private functions.MegaTag2Prep megaTag2;

    @Override
    public void init() {
        bindHardware();
        configureHardware();
        initializeLocalizationAndVisionPrep();

        telemetry.addLine("Initialized. Press PLAY.");
        telemetry.update();
    }

    @Override
    public void start() {
        shotSequence.resetToDown(leftSpindexerUp, rightSpindexerUp);
        if (driveController != null) driveController.resetForStart();
        if (megaTag2 != null) megaTag2.start();
    }

    @Override
    public void loop() {
        if (stopping) return;

        updateSensors();
        updateOperatorModes();
        updateShooterTargetVelocity();
        updateShotSequence();
        updateIntakeAndRampCommands();
        updateFlywheelCommand();
        updateDrive();
        updateTelemetry();
    }

    @Override
    public void stop() {
        stopping = true;

        FaultTolerantMecanumDrive.MotorSafety.setVelocity(leftFlyWheel, 0);
        FaultTolerantMecanumDrive.MotorSafety.setVelocity(rightFlyWheel, 0);
        FaultTolerantMecanumDrive.MotorSafety.setVelocity(rightintake, 0);

        FaultTolerantMecanumDrive.MotorSafety.setPower(rightintake, 0);
        FaultTolerantMecanumDrive.MotorSafety.setPower(leftintake, 0);

        if (driveController != null) driveController.stopSafely();
    }

    private void bindHardware() {
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        rightintake = hardwareMap.get(DcMotorEx.class, "rightintake");
        leftintake = hardwareMap.get(DcMotorEx.class, "leftintake");
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "lflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rflywheel");

        leftSpindexerUp = hardwareMap.get(Servo.class, "lspindexerup");
        rightSpindexerUp = hardwareMap.get(Servo.class, "rspindexerup");
    }

    private void configureHardware() {
        functions.setupMotors(
                leftFrontDrive,
                rightFrontDrive,
                leftBackDrive,
                rightBackDrive,
                leftFlyWheel,
                rightFlyWheel,
                rightintake,
                leftintake
        );

        functions.applyFlywheelPidf(leftFlyWheel, rightFlyWheel);

        driveController = new FaultTolerantMecanumDrive(
                leftFrontDrive,
                rightFrontDrive,
                leftBackDrive,
                rightBackDrive,
                telemetry
        );

        leftSpindexerUp.setDirection(Servo.Direction.FORWARD);
        rightSpindexerUp.setDirection(Servo.Direction.FORWARD);
    }

    private void initializeLocalizationAndVisionPrep() {
        try {
            pinpoint = new functions.PinpointIO(hardwareMap, telemetry, "odo");
            pinpoint.initializeAndConfigure();
        } catch (Exception e) {
            pinpoint = null;
            telemetry.addData("Pinpoint", "Unavailable: %s", e.getClass().getSimpleName());
        }

        megaTag2 = new functions.MegaTag2Prep(hardwareMap, telemetry, "limelight", "imu", LIMELIGHT_APRILTAG_PIPELINE);
        megaTag2.initializeAndConfigure();
    }

    private void updateSensors() {
        if (pinpoint != null) pinpoint.update();
        if (megaTag2 != null) megaTag2.update();
    }

    private void updateOperatorModes() {
        overrideMode = gamepad2.a;

        // Reuses the old flywheel Y-button slot: now toggles future vision-based targeting.
        if (gamepad1.yWasPressed()) {
            useVisionFlywheelTarget = !useVisionFlywheelTarget;
        }
    }

    private void updateShooterTargetVelocity() {
        limelightShooterDistanceInches = (megaTag2 != null) ? megaTag2.getSuggestedShooterDistanceInches() : null;
        manualFlywheelTargetVelocity = Math.max(0.0, manualFlywheelTargetVelocity);
        activeFlywheelTargetVelocity = functions.computeShooterVelocityTarget(
                shooterData,
                useVisionFlywheelTarget ? limelightShooterDistanceInches : null,
                manualFlywheelTargetVelocity
        );
    }

    private void updateShotSequence() {
        if (gamepad2.bWasPressed()) {
            shotSequence.trigger(rightintake);
        }
        shotSequence.update(rightintake, leftSpindexerUp, rightSpindexerUp);
    }

    private void updateIntakeAndRampCommands() {
        if (shotSequence.isActive()) {
            leftintake.setPower(0.0);
            return;
        }

        if (overrideMode && gamepad2.right_trigger > 0.1) {
            functions.applyRampWheelPidf(rightintake);
            rightintake.setVelocity(RAMP_WHEEL_REVERSE_VELOCITY);
            leftintake.setPower(0.0);
            return;
        }

        double intakePower = 0.0;
        if (gamepad2.left_trigger > 0.1) {
            intakePower = overrideMode ? -1.0 : 1.0;
        }

        rightintake.setPower(intakePower);
        leftintake.setPower(intakePower);
    }

    private void updateFlywheelCommand() {
        if (gamepad2.right_bumper) {
            functions.setTwoMotorsVelocity(leftFlyWheel, rightFlyWheel, activeFlywheelTargetVelocity);
        } else {
            functions.setTwoMotorsVelocity(leftFlyWheel, rightFlyWheel, 0.0);
        }
    }

    private void updateDrive() {
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        if (driveController != null) driveController.move(drive, strafe, turn);
    }

    private void updateTelemetry() {
        telemetry.addLine("--------------------------------------");
        telemetry.addData("Override Mode", overrideMode);
        telemetry.addData("Vision Flywheel Target", useVisionFlywheelTarget);
        telemetry.addData("Shooter Distance (in)", limelightShooterDistanceInches == null ? "n/a" : String.format("%.1f", limelightShooterDistanceInches));
        telemetry.addData("Flywheel Target Velocity", "%.1f", activeFlywheelTargetVelocity);
        telemetry.addData("Flywheel Target Source", useVisionFlywheelTarget ? "Vision/ShooterData" : "Manual");
        telemetry.addData("Flywheel PIDF", "P=%.1f I=%.1f D=%.1f F=%.1f",
                functions.FLYWHEEL_P, functions.FLYWHEEL_I, functions.FLYWHEEL_D, functions.FLYWHEEL_F);
        telemetry.addData("Left Wheel Velocity", "%.1f", leftFlyWheel.getVelocity());
        telemetry.addData("Right Wheel Velocity", "%.1f", rightFlyWheel.getVelocity());
        telemetry.addData("Indexer Target (deg)", "%.1f", shotSequence.getCurrentIndexerTargetDegrees());
        telemetry.addData("Shot Seq Active", shotSequence.isActive());
        if (shotSequence.isActive()) {
            telemetry.addData("Shot Seq t", "%.2f", shotSequence.getElapsedSeconds());
        }

        if (pinpoint != null) {
            telemetry.addData("Odo X (in)", "%.1f", pinpoint.getXInches());
            telemetry.addData("Odo Y (in)", "%.1f", pinpoint.getYInches());
            telemetry.addData("Odo H (deg)", "%.1f", Math.toDegrees(pinpoint.getHeadingRadians()));
        } else {
            telemetry.addData("Odo", "Unavailable");
        }

        if (megaTag2 != null) {
            megaTag2.addTelemetry(telemetry);
        }

        telemetry.update();
    }
}
