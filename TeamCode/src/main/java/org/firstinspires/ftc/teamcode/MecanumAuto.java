package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="MecanumAuto", group="Test")
public class MecanumAuto extends LinearOpMode {

    // Drive motors (same names as your TeleOp)
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    // Same ramp limiter state/behavior as your TeleOp
    private static double prevMax = 0.275;

    // Tune these if needed
    private static final double DRIVE_POWER   = 0.4; // forward power
    private static final double DRIVE_SECONDS = 0.5;  // how long to drive forward

    @Override
    public void runOpMode() {
        // ---- Motors (same hardware names) ----
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "backRightMotor");

        // ---- Directions (same as your TeleOp) ----
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // ---- Run modes (same as your TeleOp) ----
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Make sure we're stopped before start
        stopRobot();

        telemetry.addLine("Auto ready. Press PLAY.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        prevMax = 0.275; // same reset behavior as your TeleOp start()

        // --- Move out of base: drive forward for a fixed time ---
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < DRIVE_SECONDS) {
            // forward drive, no strafe, no turn
            moveRobot(DRIVE_POWER, 0.0, 0.0);

            telemetry.addData("Time", "%.2f / %.2f", timer.seconds(), DRIVE_SECONDS);
            telemetry.update();
        }

        stopRobot();

        // Optional: sit still until the auton ends
        while (opModeIsActive()) {
            idle();
        }
    }

    private void stopRobot() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    // Same mecanum mixing + ramp limiter logic as your TeleOp
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
}
