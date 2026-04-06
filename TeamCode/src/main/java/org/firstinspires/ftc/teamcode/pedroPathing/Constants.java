package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    // Keep false for current robot build: autos should not require "odo" hardware.
    public static boolean USE_PINPOINT_LOCALIZER = false;

    public static FollowerConstants followerConstants = new FollowerConstants()
            //.translationalPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0))
            .mass(12)
            .forwardZeroPowerAcceleration(-24.25)
            .lateralZeroPowerAcceleration(-47.01);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .xVelocity(57.34)
            .yVelocity(45.37)
            // Match motor directions from MecanumTeleOpTest (Right side reversed)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.0)
            .strafePodX(2.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // Encoder-only fallback localizer (uses drivetrain motor encoders, no external odo device).
    public static DriveEncoderConstants driveEncoderLocalizerConstants = new DriveEncoderConstants()
            .leftFrontMotorName("frontLeftMotor")
            .leftRearMotorName("backLeftMotor")
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            // TODO: tune with Pedro tuners for best auton accuracy.
            .forwardTicksToInches(0.022)
            .strafeTicksToInches(0.022)
            .turnTicksToInches(0.022)
            .robotWidth(12.0)
            .robotLength(8.0);

            
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        FollowerBuilder builder = new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants);

        if (USE_PINPOINT_LOCALIZER) {
            return builder.pinpointLocalizer(localizerConstants).build();
        }
        return builder.driveEncoderLocalizer(driveEncoderLocalizerConstants).build();
    }
}
