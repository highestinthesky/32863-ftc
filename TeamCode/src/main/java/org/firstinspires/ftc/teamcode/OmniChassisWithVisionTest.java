package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="OmniChassisWithVisionTest", group="Test")
public class OmniChassisWithVisionTest extends LinearOpMode {

    // Drive motors (ONLY these 4 are used)
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor TurretRotation;

    private DcMotor leftFlyWheel;
    private DcMotor rightFlyWheel;


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
        leftFlyWheel = hardwareMap.get(DcMotor.class, "lflywheel");
        rightFlyWheel = hardwareMap.get(DcMotor.class, "rflywheel");
        TurretRotation = hardwareMap.get(DcMotor.class, "turretturn");



        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFlyWheel.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurretRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        pinpoint = new PinpointIO(hardwareMap, telemetry, "odo");
        pinpoint.initializeAndConfigure();

        telemetry.addLine("Ready: 4 motors + Pinpoint (telemetry enabled)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean Rightr1Held = gamepad1.right_bumper;
            boolean Leftr1Held = gamepad1.left_bumper;
            if (Rightr1Held) {TurretRotation.setPower(1);}
            else if (Leftr1Held) {TurretRotation.setPower(-1);}
            else {TurretRotation.setPower(0);}

            boolean r2Held = gamepad1.right_trigger > 0.1;
            if (r2Held) {
                leftFlyWheel.setPower(1);
                rightFlyWheel.setPower(1);
            } else {
                leftFlyWheel.setPower(0);
                rightFlyWheel.setPower(0);
            }

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
            telemetry.addData("r2", gamepad1.right_trigger);
            telemetry.addData("lflywheel power", leftFlyWheel.getPower());
            telemetry.addData("rflywheel power", rightFlyWheel.getPower());

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
