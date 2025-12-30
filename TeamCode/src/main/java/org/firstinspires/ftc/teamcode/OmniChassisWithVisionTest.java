package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="OmniChassisWithVisionTest", group="Test")
public class OmniChassisWithVisionTest extends LinearOpMode {

    // Drive motors
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    // Other motors
    private DcMotor TurretRotation;
    private DcMotor leftFlyWheel;
    private DcMotor rightFlyWheel;
    private PinpointIO pinpoint;
    private DcMotor intake;
    private Servo spindexer;
    private Servo tservo;
    private Servo lspindexup;
    private Servo rspindexup;

    // Optional ramp limiter state
    private static double prevMax = 0.275;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---- Motors ----
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "backRightMotor");
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftFlyWheel = hardwareMap.get(DcMotor.class, "lflywheel");
        rightFlyWheel = hardwareMap.get(DcMotor.class, "rflywheel");
        spindexer = hardwareMap.get(Servo.class, "spindexer");
        tservo = hardwareMap.get(Servo.class, "tservo");
        lspindexup = hardwareMap.get(Servo.class, "lspindexup");
        rspindexup = hardwareMap.get(Servo.class, "rspindexup");

        TurretRotation = hardwareMap.get(DcMotor.class, "turretturn"); // need to configure

        // ---- Directions ----
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        rightFlyWheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);


        // ---- Run modes ----
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurretRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---- Pinpoint (placeholder – does nothing unless you add real driver code) ----
        pinpoint = new PinpointIO(hardwareMap, telemetry, "odo");
        pinpoint.initializeAndConfigure();

        telemetry.addLine("Ready: Drive + Turret + Flywheels");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Turret control (bumpers)
            if (gamepad2.right_bumper) {
                TurretRotation.setPower(1);
            } else if (gamepad2.left_bumper) {
                TurretRotation.setPower(-1);
            } else {
                TurretRotation.setPower(0);
            }
            // Flywheel control (right trigger)
            if (gamepad2.right_trigger > 0.1) {
                leftFlyWheel.setPower(1);
                rightFlyWheel.setPower(1);
            } else {
                leftFlyWheel.setPower(0);
                rightFlyWheel.setPower(0);
            }

            // Intake Control (left trigger)
            if (gamepad2.left_trigger > 0.1) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            double drive  = -gamepad1.left_stick_y;        // forward = +
            double strafe = -gamepad1.left_stick_x;        // right/left
            double turn   = -gamepad1.right_stick_x / 2.0; // slow turning

            moveRobot(drive, strafe, turn);

            // If/when you add the real Pinpoint driver, call it here
            // pinpoint.update();

            telemetry.addData("r2", "%.2f", gamepad1.right_trigger);
            telemetry.addData("lflywheel power", "%.2f", leftFlyWheel.getPower());
            telemetry.addData("rflywheel power", "%.2f", rightFlyWheel.getPower());
            telemetry.addData("Drive", "%.2f", drive);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.update();
        }
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
