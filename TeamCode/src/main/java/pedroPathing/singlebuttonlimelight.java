package pedroPathing;

import static android.os.SystemClock.elapsedRealtime;
import static pedroPathing.RandomClass.autoEnd;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled
@TeleOp(name = "singlebuttonlimelight", group = "adon")
public class singlebuttonlimelight extends OpMode {
    private Follower follower;
    private Limelight3A limelight;
    private DcMotor slide;
    private Servo claw, pivot, turn;

    // Camera and claw geometry (updated mounting)
    private final double CAM_PITCH_DEGREES = -45.0;
    private final double CAM_HEIGHT = 15.4; // inches from ground
    private final double TARGET_HEIGHT = 1.5; // inches from ground
    private final double CLAW_FORWARD_OFFSET = 1; // Claw is 1" forward of camera (negative = backwards)
    private final double CLAW_RIGHT_OFFSET = -5.5;  // Claw is 4.5" to the left of camera (negative = left)


    private double SampleDegrees; // this is the angle of the sample
    private double xDistance, yDistance, a, b; // these are for calculating target angles
    private double distanceLeftRightFromGoal, distanceForwardBackwardFromGoal; // the robot(claw)-centric right and forward offsets
    private double LRDeltaX, LRDeltaY = 0; // the field-centric movements for the right movement
    private double FBDeltaX, FBDeltaY = 0; // the field-centric movements for the forward movement
    private double backwardsDistance = 0; // if the bot needs to move backwards, this is that distance.

    private PathChain driveToGoal;
    private int state = 0;
    private long stateStart;
    private boolean dpadDownPrev = false;

    private double clawValue = 0; // 0 = closed, 0.55 = open
    private boolean optionsPrev = false;

    @Override public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.setMsTransmissionInterval(50);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(autoEnd);

        slide = hardwareMap.get(DcMotor.class, "slide");
        claw  = hardwareMap.get(Servo.class, "claw");
        pivot = hardwareMap.get(Servo.class, "pivot");
        turn  = hardwareMap.get(Servo.class, "turn");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override public void start() {
        follower.startTeleopDrive();
        follower.setMaxPower(1);
    }

    private void buildPaths() {
        Pose action = new Pose(follower.getPose().getX() + LRDeltaX + FBDeltaX, follower.getPose().getY() + LRDeltaY + FBDeltaY, follower.getPose().getHeading());
        driveToGoal = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(action)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), action.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTimeoutConstraint(400)
                .build();
    }

    @Override public void loop() {
        double robotHeading = follower.getPose().getHeading();
        boolean dpad = gamepad1.dpad_down;
        boolean targetPresent = false;

        // state 0 is when the bot is NOT running the limelight algorithm. This is a switch for claw open/close.
        if (state == 0) {
            if (gamepad1.options && !optionsPrev) {
                clawValue = (clawValue == 0 ? 0.55 : 0);
            }
            optionsPrev = gamepad1.options;
            claw.setPosition(clawValue);
        }

        // this resets the controls for teleop drive if it breaks. this does NOT reset follower coordinates/angle
        if (gamepad1.ps) {
            state = 0;
            follower.setMaxPower(1);
            follower.startTeleopDrive();
        }

        // this is for general teleop drive controls and vectors
        if (state == 0 && !follower.isBusy()) {
            follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x * 0.5,
                    true
            );
        }

        // this uses the limelight to check for "yellow" and "red" targets and allows the data to be used.
        LLResult reultsCheck = limelight.getLatestResult();
        if (reultsCheck != null && reultsCheck.isValid()) {
            for (LLResultTypes.DetectorResult dr : reultsCheck.getDetectorResults()) {
                String cls = dr.getClassName().toLowerCase();
                if (cls.equals("yellow") || cls.equals("red")) {
                    targetPresent = true;
                    double ty = dr.getTargetXDegrees();
                    double tx = -dr.getTargetYDegrees();
                    double verticalAngle = CAM_PITCH_DEGREES + ty;
                    double angleRad = Math.toRadians(verticalAngle);
                    double heightDiff = TARGET_HEIGHT - CAM_HEIGHT;
                    double lensForward = heightDiff / Math.tan(angleRad);
                    double lensRight = Math.tan(Math.toRadians(tx)) * lensForward;
                    distanceForwardBackwardFromGoal = lensForward - CLAW_FORWARD_OFFSET;
                    distanceLeftRightFromGoal = lensRight - CLAW_RIGHT_OFFSET;
                    break;
                }
            }
        }









        //THIS is the full algorithm for pickup
        if (dpad && !dpadDownPrev && state == 0 && targetPresent) {
            claw.setPosition(1); // closed

            LLResult res = limelight.getLatestResult();
            if (res != null && res.isValid()) {
                for (LLResultTypes.DetectorResult dr : res.getDetectorResults()) {
                    String cls = dr.getClassName().toLowerCase();
                    if (!cls.equals("yellow") && !cls.equals("red")) continue;
                    List<List<Double>> c = dr.getTargetCorners();
                    if (c == null || c.size() != 4) continue;

                    double ty = dr.getTargetXDegrees();
                    double tx = -dr.getTargetYDegrees();
                    double verticalAngle = CAM_PITCH_DEGREES + ty;
                    double angleRad = Math.toRadians(verticalAngle);
                    double heightDiff = TARGET_HEIGHT - CAM_HEIGHT;
                    double lensForward = heightDiff / Math.tan(angleRad);
                    double lensRight = Math.tan(Math.toRadians(tx)) * lensForward;
                    distanceForwardBackwardFromGoal = lensForward - CLAW_FORWARD_OFFSET;
                    distanceLeftRightFromGoal = lensRight - CLAW_RIGHT_OFFSET;
                    double tlx = c.get(0).get(0), tly = c.get(0).get(1);
                    double brx = c.get(2).get(0), bry = c.get(2).get(1);
                    xDistance = tlx - brx;
                    yDistance = tly - bry;
                    a = ((xDistance * 7.0/3.0) + yDistance) / (40.0/9.0);
                    b = ((yDistance * 7.0/3.0) + xDistance) / (40.0/9.0);
                    SampleDegrees = Math.toDegrees(Math.atan2(a, b));
                    break;
                }
            }

            // distance needed to travel right to target --> X Y pair movement to travel that distance robot-centric.
            if (distanceLeftRightFromGoal == 0) {
                LRDeltaX = LRDeltaY = 0;
            } else {
                LRDeltaX = distanceLeftRightFromGoal * Math.cos(-(Math.PI/2 - robotHeading));
                LRDeltaY = distanceLeftRightFromGoal * Math.sin(-(Math.PI/2 - robotHeading));
            }

            // calculate ticks for slides to extend OR distance needed to travel backwards
            int ticks = (int)Math.round(117 * distanceForwardBackwardFromGoal);

            backwardsDistance = 0;
            if (ticks > 1980) {
                backwardsDistance = (double) (ticks - 1980)/117;
            }

            // distance needed to travel forwards to target --> X Y pair movement to travel that distance robot-centric.
            FBDeltaX = backwardsDistance * Math.cos(robotHeading);
            FBDeltaY = backwardsDistance * Math.sin(robotHeading);

            buildPaths();
            follower.followPath(driveToGoal, 1, false);

            ticks = Math.min(ticks, 1980);
            slide.setTargetPosition(ticks);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);

            turn.setPosition(SampleDegrees / 180 + 0.2); // the 0.2 constant is random to add error. This is for claw turning

            state = 1;
            stateStart = elapsedRealtime();
        }

        switch (state) {
            case 1:
                if (!follower.isBusy()) {
                    pivot.setPosition(0);
                    state = 2;
                    stateStart = elapsedRealtime();
                }
                break;
            case 2:
                if (elapsedRealtime() - stateStart >= 300) {
                    claw.setPosition(0);
                    state = 3;
                    stateStart = elapsedRealtime();
                }
                break;
            case 3:
                if (elapsedRealtime() - stateStart >= 500) {
                    pivot.setPosition(1);
                    state = 4;
                    stateStart = elapsedRealtime();
                }
                break;
            case 4:
                if (elapsedRealtime() - stateStart >= 300) {
                    SampleDegrees = xDistance = yDistance = a = b = 0;
                    distanceLeftRightFromGoal = distanceForwardBackwardFromGoal = 0;
                    LRDeltaX = LRDeltaY = FBDeltaX = FBDeltaY = backwardsDistance = 0;
                    state = 0;
                    follower.setMaxPower(1);
                    follower.startTeleopDrive();
                    clawValue = 0;
                    claw.setPosition(0);
                }
                break;
        }








        dpadDownPrev = dpad;
        follower.update();

        telemetry.addData("rightDistance", distanceLeftRightFromGoal);
        telemetry.addData("forwardDistance", distanceForwardBackwardFromGoal);
        telemetry.addLine();
        telemetry.addData("Target Present", targetPresent);
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Angle of Sample", SampleDegrees);
        telemetry.addData("State", state);
        telemetry.addData("Claw Open?", clawValue > 0);
        telemetry.update();
    }

    @Override public void stop() {}
}
