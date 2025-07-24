package pedroPathing.examples;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;
@Disabled
@TeleOp(name = "Limelight Yellow And Blue", group = "adon")
public class LimelightYellowAndBlue extends OpMode {

    private Limelight3A limelight;

    double limelightMountAngleDegrees = 0;
    double limelightLensHeightInches = 12.5;
    double goalHeightInches = 1.5;

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

            // filter only red and yellow results
            List<LLResultTypes.DetectorResult> redOrYellowDetectors = new ArrayList<>();
            for (LLResultTypes.DetectorResult dr : detectors) {
                String className = dr.getClassName().toLowerCase();
                if (className.equals("yellow") || className.equals("blue")) {
                    redOrYellowDetectors.add(dr);
                }
            }

            if (!redOrYellowDetectors.isEmpty()) {
                for (LLResultTypes.DetectorResult dr : redOrYellowDetectors) {
                    double limelightDistanceLeftRightToGoal = (11)*(Math.tan(Math.toRadians(dr.getTargetYDegrees())))+1;
                    double limelightDistanceForwardBackwardToGoal = (11)*(Math.tan(-1*(Math.toRadians(dr.getTargetXDegrees()))))+1;

                    telemetry.addData("Class", dr.getClassName());
                    telemetry.addData("DistanceLeftRight", (int)limelightDistanceLeftRightToGoal);
                    telemetry.addData("DistanceForwardBackwardToGoal", (int)limelightDistanceForwardBackwardToGoal);
                    telemetry.addData("Area", "%.2f", dr.getTargetArea());
                    telemetry.addData("X°", "%.2f", dr.getTargetXDegrees());
                    telemetry.addData("Y°", "%.2f", dr.getTargetYDegrees());
                    telemetry.addData("confidence", dr.getConfidence());
                    telemetry.addData("getYpixels", dr.getTargetYPixels());
                    telemetry.addData("getXpixels", dr.getTargetXPixels());
                    telemetry.addData("getCorners", dr.getTargetCorners());
                }
            } else {
                telemetry.addData("NoBlueOrYellowFound", 1);
                telemetry.update();
            }
            telemetry.update();
        }
        telemetry.update();
    }
}
