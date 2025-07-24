package pedroPathing.examples;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "reworkingLimelight", group = "adon")
public class reworkingLimelight extends OpMode {

    private Limelight3A limelight;

    // Constants for geometry
    private final double camPitchDegrees = -45.0; // Camera is pitched 45° downward from horizontal
    private final double camHeight = 15.4;        // Limelight lens height from the floor (in inches)
    private final double targetHeight = 1.5;      // Goal object height from the floor (in inches)
    private final double FORWARD_ERROR_MULTIPLIER = 1.00;   // Adjust this based on testing
    private final double SIDE_ERROR_MULTIPLIER = 1.00;      // Adjust this based on testing
    private final double CLAW_FORWARD_OFFSET = 0;   // Inches from lens to claw in forward/backward direction (+ means claw is ahead of lens)
    private final double CLAW_RIGHT_OFFSET = -4.2;     // Inches from lens to claw in right/left direction (+ means claw is right of lens)




    // Distance from lens to target
    private double lensleftRightDistance = 0;
    private double lensforwardBackwardDistance = 0;

    // Distance from claw to target (adjusted for offset)
    private double clawleftRightDistance = 0;
    private double clawforwardBackwardDistance = 0;

    @Override
    public void init() {
        // Initialize Limelight and set pipeline
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.setMsTransmissionInterval(500);
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            for (LLResultTypes.DetectorResult dr : result.getDetectorResults()) {
                String cls = dr.getClassName().toLowerCase();

                if (!cls.equals("yellow") && !cls.equals("red")) continue;

                double ty = dr.getTargetXDegrees(); // vertical angle
                double tx = -dr.getTargetYDegrees(); // horizontal angle (inverted to match typical coordinate logic)

                double angleToTargetDeg = camPitchDegrees + ty;
                double angleToTargetRad = Math.toRadians(angleToTargetDeg);
                double heightDiff = targetHeight - camHeight;

                // Compute lens-relative distances
                lensforwardBackwardDistance = (heightDiff / Math.tan(angleToTargetRad)) * FORWARD_ERROR_MULTIPLIER;
                lensleftRightDistance = (Math.tan(Math.toRadians(tx)) * lensforwardBackwardDistance) * SIDE_ERROR_MULTIPLIER;

                // Compute claw-relative distances by applying fixed offset
                clawforwardBackwardDistance = lensforwardBackwardDistance - CLAW_FORWARD_OFFSET;
                clawleftRightDistance = lensleftRightDistance - CLAW_RIGHT_OFFSET;

                // Telemetry output
                telemetry.addData("Target", cls);
                telemetry.addData("Lens FORWARD Dist (in)", String.format("%.2f", lensforwardBackwardDistance));
                telemetry.addData("Lens RIGHT Dist (in)", String.format("%.2f", lensleftRightDistance));
                telemetry.addData("Claw FORWARD Dist (in)", String.format("%.2f", clawforwardBackwardDistance));
                telemetry.addData("Claw RIGHT Dist (in)", String.format("%.2f", clawleftRightDistance));
            }
        } else {
            telemetry.addLine("No valid Limelight result.");
        }

        telemetry.update();
    }

    @Override
    public void stop() {}
}
