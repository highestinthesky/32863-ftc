package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Minimal OmniChassis using ONLY:
 *  - 4 drive motors
 *  - goBILDA Pinpoint Odometry Computer on I2C (device name: "odometry")
 *
 * Adds:
 *  1) Pose2d class
 *  2) PinpointIO.getPose() (currently MOCK integration so it works immediately)
 *  3) Constant telemetry update showing robot "location" on Driver Hub
 *
 * NOTE: Once you install/confirm the actual Pinpoint driver class + methods,
 * replace the MOCK in PinpointIO.update/getPose with real readings.
 */
@TeleOp(name="OmniChassisWithVisionTest", group="Test")
public class OmniChassisWithVisionTest extends LinearOpMode {

    // Drive motors (ONLY these 4 are used)
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    // Pinpoint odometry computer (I2C) device name: "odometry"
    private PinpointIO pinpoint;

    // Optional ramp limiter state
    private static double prevMax = 0.275;

    /** Simple pose container for telemetry (inches + degrees). */
    public static class Pose2d {
        public double xInches;     // +right (strafe)
        public double yInches;     // +forward
        public double headingDeg;  // +CCW (mock)

        public Pose2d(double x, double y, double h) {
            xInches = x;
            yInches = y;
            headingDeg = h;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // ---- Drive motors ----
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "backRightMotor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---- Pinpoint (still MOCK inside PinpointIO) ----
        pinpoint = new PinpointIO(hardwareMap, telemetry, "odometry");
        pinpoint.initializeAndConfigure();

        telemetry.addLine("Ready: 4 motors + Pinpoint (telemetry enabled)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Read sticks
            double drive  = -gamepad1.left_stick_y;        // forward = +
            double strafe = -gamepad1.left_stick_x;        // right? depends on your preference; kept from your code
            double turn   = -gamepad1.right_stick_x / 2.0; // slow turning

            // Drive robot
            moveRobot(drive, strafe, turn);

            // Update odometry (MOCK integration for now)
            pinpoint.update(drive, strafe, turn);
            Pose2d pose = pinpoint.getPose();

            // Constant telemetry
            telemetry.addData("POSE X (in)", "%.2f", pose.xInches);
            telemetry.addData("POSE Y (in)", "%.2f", pose.yInches);
            telemetry.addData("HEADING (deg)", "%.1f", pose.headingDeg);

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
     * Pinpoint wrapper.
     *
     * Right now it uses a MOCK "dead reckoning" integrator so:
     *  - telemetry changes while you drive
     *  - you confirm everything is wired & OpMode runs
     *
     * Replace the mock with your actual goBILDA Pinpoint Java driver calls later.
     */
    private static class PinpointIO {
        private final Telemetry telemetry;
        private final Object driver;

        // MOCK pose state
        private double mockXIn = 0.0;
        private double mockYIn = 0.0;
        private double mockHeadingDeg = 0.0;

        PinpointIO(HardwareMap hardwareMap, Telemetry telemetry, String deviceName) {
            this.telemetry = telemetry;

            // This keeps your code compiling now, but it is NOT the real Pinpoint driver.
            // Replace Object with the actual driver class once you have it in your project.
            driver = hardwareMap.get(Object.class, deviceName);
        }

        void initializeAndConfigure() {
            telemetry.addLine("Pinpoint: Initialized (MOCK pose for telemetry)");
            telemetry.update();
        }

        /**
         * MOCK update:
         * - integrates joystick commands into a fake pose, just for visible telemetry.
         * - Not real odometry.
         */
        void update(double drive, double strafe, double turn) {
            // Scale factors chosen to look reasonable on screen; tweak if you like.
            mockXIn += strafe * 0.15;       // inches per loop-ish
            mockYIn += drive * 0.15;
            mockHeadingDeg += turn * 6.0;   // deg per loop-ish

            // keep heading in (-180, 180]
            while (mockHeadingDeg > 180) mockHeadingDeg -= 360;
            while (mockHeadingDeg <= -180) mockHeadingDeg += 360;
        }

        Pose2d getPose() {
            return new Pose2d(mockXIn, mockYIn, mockHeadingDeg);
        }
    }
}
