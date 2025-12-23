package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "OmniChassis Field-Centric (Pinpoint)", group = "Test")
public class OmniChassisFieldCentricTest extends LinearOpMode {

    private DcMotor lf, rf, lb, rb;
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {

        // ---------------- Motors ----------------
        lf = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rf = hardwareMap.get(DcMotor.class, "frontRightMotor");
        lb = hardwareMap.get(DcMotor.class, "backLeftMotor");
        rb = hardwareMap.get(DcMotor.class, "backRightMotor");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---------------- Pinpoint ----------------
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // The Pinpoint API requires a DistanceUnit as the third argument.
        // xOffset: distance sideways from center to X (forward) pod.
        // yOffset: distance forward from center to Y (strafe) pod.
        odo.setOffsets(-15.875, -101.6, DistanceUnit.MM);

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        odo.resetPosAndIMU();

        telemetry.addLine("Field-centric ready (Pinpoint)");
        telemetry.addLine("Press A to re-zero");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            Pose2D pose = odo.getPosition();

            double headingRad = pose.getHeading(AngleUnit.RADIANS);

            // Driver input (field frame)
            double yField = -gamepad1.left_stick_y;
            double xField =  gamepad1.left_stick_x;
            double turn   =  gamepad1.right_stick_x;

            double speed = gamepad1.left_bumper ? 0.35 : 1.0;

            // Field -> Robot transform
            double cos = Math.cos(headingRad);
            double sin = Math.sin(headingRad);

            double xRobot =  xField * cos + yField * sin;
            double yRobot = -xField * sin + yField * cos;

            driveMecanum(yRobot * speed, xRobot * speed, turn * speed);

            if (gamepad1.a) {
                odo.resetPosAndIMU();
            }

            telemetry.addData("X (mm)", "%.1f", pose.getX(DistanceUnit.MM));
            telemetry.addData("Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
            telemetry.addData("Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }

    private void driveMecanum(double drive, double strafe, double turn) {
        double lfP = drive + strafe + turn;
        double rfP = drive - strafe - turn;
        double lbP = drive - strafe + turn;
        double rbP = drive + strafe - turn;

        double max = Math.max(
                Math.max(Math.abs(lfP), Math.abs(rfP)),
                Math.max(Math.abs(lbP), Math.abs(rbP))
        );

        if (max > 1.0) {
            lfP /= max; rfP /= max; lbP /= max; rbP /= max;
        }

        lf.setPower(Range.clip(lfP, -1, 1));
        rf.setPower(Range.clip(rfP, -1, 1));
        lb.setPower(Range.clip(lbP, -1, 1));
        rb.setPower(Range.clip(rbP, -1, 1));
    }
}
