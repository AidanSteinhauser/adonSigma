package pedroPathing.examples;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
@Disabled
@TeleOp(name = "Limelight Yellow And Red", group = "adon")
public class LimelightYellowAndRed extends OpMode {

    private Limelight3A limelight;
    private DcMotor slide = null;
    private Servo claw = null;
    private Servo pivot = null;
    private Servo turn = null;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Use the detector pipeline
        limelight.start();
        telemetry.setMsTransmissionInterval(11);
        slide = hardwareMap.get(DcMotor.class, "slide");
        claw = hardwareMap.get(Servo.class, "claw");
        pivot = hardwareMap.get(Servo.class, "pivot");
        turn = hardwareMap.get(Servo.class, "turn");
        pivot.setPosition(0);
        turn.setPosition(0);
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectors = result.getDetectorResults();

            boolean foundTarget = false;

            // Filter only red and yellow results
            for (LLResultTypes.DetectorResult dr : detectors) {
                String className = dr.getClassName().toLowerCase();
                if (className.equals("yellow") || className.equals("red")) {
                    foundTarget = true;

                    // Get the target corners
                    List<List<Double>> corners = dr.getTargetCorners();

                    if (corners != null && corners.size() == 4) {
                        // Extracting corner coordinates (order: top-left, top-right, bottom-left, bottom-right)
                        double topLeftX = corners.get(0).get(0);
                        double topLeftY = corners.get(0).get(1);
                        double topRightX = corners.get(1).get(0);
                        double topRightY = corners.get(1).get(1);
                        double bottomRightX = corners.get(2).get(0);
                        double bottomRightY = corners.get(2).get(1);
                        double bottomLeftX = corners.get(3).get(0);
                        double bottomLeftY = corners.get(3).get(1);

                        // Swapping bottom-left and bottom-right for correct coordinates
                        telemetry.addData("Top Left", "X: %.2f, Y: %.2f", topLeftX, topLeftY);
                        telemetry.addData("Top Right", "X: %.2f, Y: %.2f", topRightX, topRightY);
                        telemetry.addData("BottomRight", "X: %.2f, Y: %.2f", bottomRightX, bottomRightY);  // swapped
                        telemetry.addData("BottomLeft", "X: %.2f, Y: %.2f", bottomLeftX, bottomLeftY);  // swapped

                        // Calculate the X and Y distances between the top-left and bottom-left
                        double xDistance = bottomRightX - topLeftX;
                        double yDistance = bottomRightY - topLeftY;

                        double a = ((((xDistance * -7.0 / 3.0) + yDistance)) / (-40.0 / 9.0));
                        double b = ((((yDistance * -7.0 / 3.0) + xDistance)) / (-40.0 / 9.0));

                        double SampleDegrees = Math.toDegrees(Math.atan2(a, b));
                        turn.setPosition(SampleDegrees/180);
                        // Output distance calculations
                        telemetry.addData("X Distance", xDistance);
                        telemetry.addData("Y Distance", yDistance);

                        // Add back other telemetry data
                        telemetry.addData("Class", dr.getClassName());
                        telemetry.addData("A", (a));
                        telemetry.addData("B", (b));
                        telemetry.addData("SampleDegrees", "%.2f", SampleDegrees);
                        telemetry.addData("Area", "%.2f", dr.getTargetArea());
                        telemetry.addData("X°", "%.2f", dr.getTargetXDegrees());
                        telemetry.addData("Y°", "%.2f", dr.getTargetYDegrees());
                        telemetry.addData("Confidence", "%.2f", dr.getConfidence());
                        telemetry.addData("Y Pixels", dr.getTargetYPixels());
                        telemetry.addData("X Pixels", dr.getTargetXPixels());
                        telemetry.addData("Corners Raw", dr.getTargetCorners());
                    } else {
                        telemetry.addData("Corners", "Invalid number of corners or null");
                    }
                }
            }

            if (!foundTarget) {
                telemetry.addData("No Target", "No red or yellow targets found");
            }
        } else {
            telemetry.addData("No Result", "Limelight result invalid or null");
        }

        telemetry.update();
    }
}
