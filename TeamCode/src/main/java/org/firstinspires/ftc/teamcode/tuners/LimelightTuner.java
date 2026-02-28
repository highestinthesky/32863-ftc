package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "LimelightTuner", group = "Tuners")
public class LimelightTuner extends OpMode {
    public static String LIMELIGHT_NAME = "limelight";
    public static String IMU_NAME = "imu";
    public static int PIPELINE_ID = 1;
    public static int POLL_RATE_HZ = 50;

    private Limelight3A limelight;
    private IMU imu;
    private String initError;
    private LLResultTypes.CalibrationResult latestCalibration;

    @Override
    public void init() {
        initError = null;
        latestCalibration = null;

        try {
            limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
            imu = hardwareMap.get(IMU.class, IMU_NAME);

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            );
            imu.initialize(new IMU.Parameters(orientationOnRobot));

            limelight.setPollRateHz(POLL_RATE_HZ);
            limelight.pipelineSwitch(PIPELINE_ID);
            latestCalibration = limelight.getCalLatest();
        } catch (Exception e) {
            initError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }

        updateTelemetry();
    }

    @Override
    public void start() {
        if (limelight != null && initError == null) {
            limelight.start();
        }
    }

    @Override
    public void loop() {
        if (limelight != null && imu != null && initError == null) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            latestCalibration = limelight.getCalLatest();
        }
        updateTelemetry();
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }

    private void updateTelemetry() {
        LLStatus status = (limelight != null) ? limelight.getStatus() : null;
        LLResult result = (limelight != null) ? limelight.getLatestResult() : null;

        String pipelineType = status != null ? status.getPipelineType() : "unknown";
        boolean isAprilTagPipeline = pipelineType != null && pipelineType.toLowerCase().contains("apriltag");
        boolean calibrationLoaded = latestCalibration != null && latestCalibration.isValid();

        Pose3D mt2Pose = (result != null && result.isValid()) ? result.getBotpose_MT2() : null;
        boolean has3dPose = mt2Pose != null && result.getBotposeTagCount() > 0;

        // SDK does not expose a direct SolvePnP enabled flag.
        // Infer readiness from valid fiducials + available 3D botpose output.
        boolean solvePnpLikelyReady = result != null
                && result.isValid()
                && result.getFiducialResults() != null
                && !result.getFiducialResults().isEmpty()
                && has3dPose;

        telemetry.addLine("Limelight Tuner");
        telemetry.addData("Limelight Name", LIMELIGHT_NAME);
        telemetry.addData("IMU Name", IMU_NAME);
        telemetry.addData("Init Error", initError == null ? "none" : initError);
        telemetry.addData("Connected", limelight != null && limelight.isConnected());
        telemetry.addData("Running", limelight != null && limelight.isRunning());
        telemetry.addData("Expected Pipeline", PIPELINE_ID);
        telemetry.addData("Active Pipeline", status != null ? status.getPipelineIndex() : -1);
        telemetry.addData("Pipeline Type", pipelineType);
        telemetry.addData("AprilTag Pipeline", isAprilTagPipeline ? "OK" : "NOT APRILTAG");

        telemetry.addData("Calibration Loaded", calibrationLoaded);
        if (latestCalibration != null) {
            telemetry.addData("Calibration Name", latestCalibration.getDisplayName());
            telemetry.addData("Reprojection Err", "%.4f", latestCalibration.getReprojectionError());
        } else {
            telemetry.addData("Calibration Name", "none");
        }

        telemetry.addData("Result Valid", result != null && result.isValid());
        telemetry.addData("Tag Count", result != null ? result.getBotposeTagCount() : 0);
        telemetry.addData("3D Pose Estimation", has3dPose ? "OK" : "NO 3D POSE");
        telemetry.addData("SolvePnP", solvePnpLikelyReady ? "LIKELY READY" : "NOT VERIFIED");

        if (result != null && result.isValid()) {
            telemetry.addData("tx", "%.2f", result.getTx());
            telemetry.addData("ty", "%.2f", result.getTy());
            telemetry.addData("ta", "%.2f", result.getTa());
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                telemetry.addData("Primary Tag ID", result.getFiducialResults().get(0).getFiducialId());
            }
        } else {
            telemetry.addData("tx", "n/a");
            telemetry.addData("ty", "n/a");
            telemetry.addData("ta", "n/a");
        }

        telemetry.update();
    }
}
