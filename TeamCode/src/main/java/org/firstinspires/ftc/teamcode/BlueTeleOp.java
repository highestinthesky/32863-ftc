package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.shooter.AutoShotSequenceController;
import org.firstinspires.ftc.teamcode.shooter.ShooterData;
import org.firstinspires.ftc.teamcode.shooter.ShotControlConfig;
import org.firstinspires.ftc.teamcode.shooter.TurretCrServoController;

@TeleOp(name = "BlueTeleOp")
public class BlueTeleOp extends OpMode {
    private static final int GOAL_TAG_ID = 20;
    private static final int LIMELIGHT_APRILTAG_PIPELINE = 21;
    private static final String TURRET_RIGHT_SERVO = "rightturretturn";
    private static final String TURRET_LEFT_SERVO = "leftturretturn";

    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightBackDrive;
    private FaultTolerantMecanumDrive driveController;

    private DcMotorEx rightintake;
    private DcMotorEx leftintake;
    private DcMotorEx leftFlyWheel;
    private DcMotorEx rightFlyWheel;

    private boolean stopping = false;
    private boolean overrideMode = false;

    private final ShotControlConfig shotConfig = ShotControlConfig.defaultConfig();
    private final ShooterData shooterData = ShooterData.defaultTuned();

    private functions.PinpointIO pinpoint;
    private functions.MegaTag2Prep megaTag2;
    private TurretCrServoController turretController;
    private AutoShotSequenceController autoShotController;

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
        if (driveController != null) driveController.resetForStart();
        if (megaTag2 != null) megaTag2.start();
    }

    @Override
    public void loop() {
        if (stopping) return;

        updateSensors();
        updateOperatorModes();
        updateAutoShotSequence();
        updateIntakeCommands();
        updateDrive();
        updateTelemetry();
    }

    @Override
    public void stop() {
        stopping = true;

        if (autoShotController != null) {
            autoShotController.stopAll();
        } else {
            FaultTolerantMecanumDrive.MotorSafety.setVelocity(leftFlyWheel, 0);
            FaultTolerantMecanumDrive.MotorSafety.setVelocity(rightFlyWheel, 0);
            FaultTolerantMecanumDrive.MotorSafety.setVelocity(rightintake, 0);
            FaultTolerantMecanumDrive.MotorSafety.setVelocity(leftintake, 0);
            FaultTolerantMecanumDrive.MotorSafety.setPower(rightintake, 0);
            FaultTolerantMecanumDrive.MotorSafety.setPower(leftintake, 0);
        }

        if (turretController != null) turretController.stop();

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

        driveController = new FaultTolerantMecanumDrive(
                leftFrontDrive,
                rightFrontDrive,
                leftBackDrive,
                rightBackDrive,
                telemetry
        );
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

        turretController = new TurretCrServoController(hardwareMap, TURRET_RIGHT_SERVO, TURRET_LEFT_SERVO);
        autoShotController = new AutoShotSequenceController(
                leftFlyWheel,
                rightFlyWheel,
                rightintake,
                leftintake,
                megaTag2,
                shooterData,
                GOAL_TAG_ID,
                turretController,
                shotConfig
        );
    }

    private void updateSensors() {
        if (pinpoint != null) pinpoint.update();
        if (megaTag2 != null) megaTag2.update();
    }

    private void updateOperatorModes() {
        overrideMode = gamepad2.a;
    }

    private void updateAutoShotSequence() {
        if (autoShotController != null) {
            autoShotController.update(gamepad2.bWasPressed(), gamepad2.xWasPressed());
        }
    }

    private void updateIntakeCommands() {
        if (autoShotController != null && autoShotController.isBusy()) {
            return;
        }

        if (overrideMode && gamepad2.right_trigger > shotConfig.triggerDeadband) {
            rightintake.setVelocity(shotConfig.intakeReverseVelocity);
            leftintake.setPower(0.0);
            return;
        }

        double intakePower = 0.0;
        if (gamepad2.left_trigger > shotConfig.triggerDeadband) {
            intakePower = overrideMode ? shotConfig.intakeReversePower : shotConfig.intakeCollectPower;
        }

        rightintake.setPower(intakePower);
        leftintake.setPower(intakePower);
    }

    private void updateDrive() {
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        if (driveController != null) driveController.move(drive, strafe, turn);
    }

    private void updateTelemetry() {
        telemetry.addLine("--------------------------------------");
        telemetry.addData("Alliance", "Blue");
        telemetry.addData("Goal Tag ID", GOAL_TAG_ID);
        telemetry.addData("Override Mode", overrideMode);
        telemetry.addData("Shoot Buttons", "B=2-ball, X=3-ball");
        telemetry.addData("Flywheel PIDF", "P=%.1f I=%.1f D=%.1f F=%.1f",
                functions.FLYWHEEL_P, functions.FLYWHEEL_I, functions.FLYWHEEL_D, functions.FLYWHEEL_F);
        telemetry.addData("Left Wheel Velocity", "%.1f", leftFlyWheel.getVelocity());
        telemetry.addData("Right Wheel Velocity", "%.1f", rightFlyWheel.getVelocity());
        telemetry.addData("Intake Front Vel", "%.1f", rightintake.getVelocity());
        telemetry.addData("Intake Transfer Vel", "%.1f", leftintake.getVelocity());

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
        if (autoShotController != null) {
            autoShotController.addTelemetry(telemetry);
        }

        telemetry.update();
    }
}
