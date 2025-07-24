package pedroPathing.examples;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;
@Disabled
@TeleOp(name = "Limelight Detector Telemetry", group = "adon")
public class LimelightSmartTelemetry extends OpMode {

    private Limelight3A limelight;

    double limelightMountAngleDegrees = 0;
    double limelightLensHeightInches = 4.25;
    double goalHeightInches = 1;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Use the detector pipeline
        limelight.start();
        telemetry.setMsTransmissionInterval(11);
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectors = result.getDetectorResults();
            if (!detectors.isEmpty()) {
                for (LLResultTypes.DetectorResult dr : detectors) {

                    double angleToGoalDegrees = limelightMountAngleDegrees + result.getTy();
                    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(Math.toRadians(angleToGoalDegrees)) + 0.75;
                    telemetry.addData("distance", ((int) distanceFromLimelightToGoalInches));
                    telemetry.addData("Class", dr.getClassName());
                    telemetry.addData("Area", "%.2f", dr.getTargetArea());
                    telemetry.addData("X°", "%.2f", dr.getTargetXDegrees());
                    telemetry.addData("Y°", "%.2f", dr.getTargetYDegrees());
                    telemetry.addData("confidence", dr.getConfidence());
                    telemetry.addData("getYpixels", dr.getTargetYPixels());
                    telemetry.addData("getXpixels", dr.getTargetXPixels());
                    telemetry.addData("getCorners", dr.getTargetCorners());
                }
            } else {
                telemetry.addLine("No detectors found.");
            }
        } else {
            telemetry.addLine("No valid result from Limelight.");
        }

        telemetry.update();
    }
}