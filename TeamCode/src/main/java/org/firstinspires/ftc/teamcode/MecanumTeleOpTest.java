package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class MecanumTeleOpTest extends OpMode {

    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightBackDrive;
    private DcMotorEx frontIntake;
    private DcMotorEx doubleIntake;

    private static final double P = 3.0;
    private static final double I = 0.0;
    private static final double D = 0.0;
    private static final double F = 7.0;

    private DcMotorEx rightFlyWheel;
    private DcMotorEx leftFlyWheel;
    double curvelocity = 0;

    // Ramp limiter state
    private double prevMax = 0.0;

    // Fault event message state
    private String faultEventMsg = "";
    private int faultEventLoopsLeft = 0;

    private static final double CMD_MIN = 0.35;
    private static final double VEL_ABS_MIN = 120.0;  // ticks/sec
    private static final double VEL_REL_RATIO = 0.20;
    private static final int FAIL_LOOPS = 8;
    private static final int RECOVER_LOOPS = 15;

    private static class MotorHealth {
        int badLoops = 0;
        int goodLoops = 0;
        boolean failed = false;
    }

    private final MotorHealth lfHealth = new MotorHealth();
    private final MotorHealth rfHealth = new MotorHealth();
    private final MotorHealth lbHealth = new MotorHealth();
    private final MotorHealth rbHealth = new MotorHealth();

    @Override
    public void init() {
        // chassis motors setup
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        doubleIntake = hardwareMap.get(DcMotorEx.class, "doubleintake");
        frontIntake = hardwareMap.get(DcMotorEx.class, "frontintake");

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


        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "lflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rflywheel");
        // flywheel directions
        rightFlyWheel.setDirection(DcMotorEx.Direction.REVERSE);
        leftFlyWheel.setDirection(DcMotorEx.Direction.FORWARD);
        // flywheel encoder setup
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // flywheel PIDF setup
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        leftFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFlyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        double drive  = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn   = -gamepad1.right_stick_x;

        moveRobotFaultTolerant(drive, strafe, turn);

        if (gamepad1.right_bumper){
            frontIntake.setPower(1);
            doubleIntake.setPower(1);
        } else {
            frontIntake.setPower(0);
            doubleIntake.setPower(0);
        }
        if (gamepad1.left_bumper) {
            curvelocity = 6000;
        }else {
            curvelocity = 0;
        }
        leftFlyWheel.setVelocity(curvelocity);
        rightFlyWheel.setVelocity(curvelocity);

        telemetry.update();
    }

    private void updateHealth(MotorHealth h, DcMotorEx m, double cmd, double vel, double maxVel) {
        // If command is small, or nobody is moving much, don't accumulate fault counters.
        if (Math.abs(cmd) < CMD_MIN || maxVel < VEL_ABS_MIN) {
            h.badLoops = 0;
            h.goodLoops = 0;
            return;
        }

        double threshold = Math.max(VEL_ABS_MIN, maxVel * VEL_REL_RATIO);
        boolean moving = vel >= threshold;

        if (!moving) {
            h.badLoops++;
            h.goodLoops = 0;
            if (h.badLoops >= FAIL_LOOPS) {
                if (!h.failed) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                h.failed = true;
            }
        } else {
            h.goodLoops++;
            h.badLoops = 0;
            if (h.failed && h.goodLoops >= RECOVER_LOOPS) {
                h.failed = false;
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }

    public void moveRobotFaultTolerant(double drive, double strafe, double turn) {
        // Remember previous failure states (so we can detect "just failed")
        boolean prevLfFailed = lfHealth.failed;
        boolean prevRfFailed = rfHealth.failed;
        boolean prevLbFailed = lbHealth.failed;
        boolean prevRbFailed = rbHealth.failed;

        // --- Measure current velocities ---
        double vLF = Math.abs(leftFrontDrive.getVelocity());
        double vRF = Math.abs(rightFrontDrive.getVelocity());
        double vLB = Math.abs(leftBackDrive.getVelocity());
        double vRB = Math.abs(rightBackDrive.getVelocity());
        double maxV = Math.max(Math.max(vLF, vRF), Math.max(vLB, vRB));

        // --- Normal 4-wheel powers (your exact mapping) ---
        double lf = drive - strafe - turn;
        double rf = drive + strafe + turn;
        double lb = drive + strafe - turn;
        double rb = drive - strafe + turn;

        // Normalize these for detection consistency
        double detMax = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb)));
        if (detMax > 1.0) {
            lf /= detMax; rf /= detMax; lb /= detMax; rb /= detMax;
        }

        // --- Update health flags ---
        updateHealth(lfHealth, leftFrontDrive,  lf, vLF, maxV);
        updateHealth(rfHealth, rightFrontDrive, rf, vRF, maxV);
        updateHealth(lbHealth, leftBackDrive,   lb, vLB, maxV);
        updateHealth(rbHealth, rightBackDrive,  rb, vRB, maxV);

        boolean lfOk = !lfHealth.failed;
        boolean rfOk = !rfHealth.failed;
        boolean lbOk = !lbHealth.failed;
        boolean rbOk = !rbHealth.failed;

        int failedCount = (lfOk ? 0 : 1) + (rfOk ? 0 : 1) + (lbOk ? 0 : 1) + (rbOk ? 0 : 1);

        // --- If a motor JUST failed this loop, create the requested telemetry message ---
        if (!prevLfFailed && lfHealth.failed) {
            faultEventMsg = "Front Left motor is currently offline and the system has adjusted.";
            faultEventLoopsLeft = 30;
        } else if (!prevRfFailed && rfHealth.failed) {
            faultEventMsg = "Front Right motor is currently offline and the system has adjusted.";
            faultEventLoopsLeft = 30;
        } else if (!prevLbFailed && lbHealth.failed) {
            faultEventMsg = "Back Left motor is currently offline and the system has adjusted.";
            faultEventLoopsLeft = 30;
        } else if (!prevRbFailed && rbHealth.failed) {
            faultEventMsg = "Back Right motor is currently offline and the system has adjusted.";
            faultEventLoopsLeft = 30;
        }

        // --- If exactly ONE motor failed, remap to 3-wheel solution ---
        if (failedCount == 1) {
            double d = drive, s = strafe, t = turn;

            if (!lfOk) {
                lf = 0;
                rf = 2 * (s + t);
                lb = 2 * (d - t);
                rb = 2 * (d - s);
            } else if (!rfOk) {
                rf = 0;
                lf = -2 * (s + t);
                lb = 2 * (d + s);
                rb = 2 * (d + t);
            } else if (!lbOk) {
                lb = 0;
                lf = 2 * (d - t);
                rf = 2 * (d + s);
                rb = 2 * (t - s);
            } else { // !rbOk
                rb = 0;
                lf = 2 * (d - s);
                rf = 2 * (d + t);
                lb = 2 * (s - t);
            }
        }

        // --- Normalize final outputs ---
        double maxAbs = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb)));
        if (maxAbs > 1.0) {
            lf /= maxAbs; rf /= maxAbs; lb /= maxAbs; rb /= maxAbs;
            maxAbs = 1.0;
        }

        // --- Ramp limiter ---
        double desired = maxAbs; // 0..1
        double allowed = desired;

        if (allowed > prevMax + 0.075) allowed = prevMax + 0.075;
        if (allowed < 0) allowed = 0;
        if (allowed > 1) allowed = 1;

        double scale = (desired > 1e-6) ? (allowed / desired) : 0.0;
        prevMax = allowed;

        lf *= scale; rf *= scale; lb *= scale; rb *= scale;

        // --- Apply powers ---
        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);

        // --- Telemetry output ---
        if (faultEventLoopsLeft > 0) {
            telemetry.addLine(faultEventMsg);
            faultEventLoopsLeft--;
        }

        if (failedCount > 0) {
            String offline =
                    (!lfOk ? "Front Left" :
                            !rfOk ? "Front Right" :
                                    !lbOk ? "Back Left" :
                                            "Back Right");
            telemetry.addLine(offline + " motor is currently offline (fault mode active).");
        }

        telemetry.addData("DriveVel", "LF %.0f RF %.0f LB %.0f RB %.0f", vLF, vRF, vLB, vRB);
    }
}