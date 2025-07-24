package pedroPathing.examples;

import static android.os.SystemClock.sleep;
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
import com.pedropathing.util.Timer;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled
@TeleOp(name = "adderswitch", group = "adon")
public class adderswitch extends OpMode {
    private Follower follower;
    private Limelight3A limelight;
    private DcMotor slide;
    private Servo claw, pivot, turn;
    private Timer opmodeTimer;

    // —— class fields to hold the last snapshot ——
    private double SampleDegrees;
    private double xDistance;
    double LRDeltaX;
    double LRDeltaY;
    private double yDistance;
    private double a;
    private double b;
    private double distanceLeftRightFromGoal;
    private double distanceForwardBackwardFromGoal;
    private PathChain driveToGoal;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.setMsTransmissionInterval(11);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(autoEnd);

        slide = hardwareMap.get(DcMotor.class, "slide");
        claw  = hardwareMap.get(Servo.class, "claw");
        pivot = hardwareMap.get(Servo.class, "pivot");
        turn  = hardwareMap.get(Servo.class, "turn");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        buildPaths();
    }
    public void buildPaths() {
        Pose lefttemp = new Pose(follower.getPose().getX() + LRDeltaX, follower.getPose().getY() + LRDeltaY, follower.getPose().getHeading());
        driveToGoal = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(lefttemp)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), lefttemp.getHeading())
                .build();

    }

    @Override public void init_loop() { }

    @Override public void start() {
        follower.startTeleopDrive();
        opmodeTimer.resetTimer();
        pivot.setPosition(1);
        claw.setPosition(1);
    }

    @Override
    public void loop() {
        // drive

        Pose lefttemp = new Pose(follower.getPose().getX() + LRDeltaX, follower.getPose().getY() + LRDeltaY, follower.getPose().getHeading());

        if (!follower.isBusy()) {
            follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x * 0.5,
                    true
            );
        }

        // ** declare these here so they exist for telemetry below **

        if (distanceLeftRightFromGoal == 0) {
            LRDeltaX = 0;
            LRDeltaY = 0;
        } else {
            LRDeltaX = (distanceLeftRightFromGoal + 4.5)
                    * Math.cos(-((Math.PI / 2) - follower.getPose().getHeading()));
            LRDeltaY = (distanceLeftRightFromGoal + 4.5)
                    * Math.sin(-((Math.PI / 2) - follower.getPose().getHeading()));
        }
        if (gamepad1.ps) {
            follower.breakFollowing();
            follower.setMaxPower(1.0);
            follower.startTeleopDrive();
        }

        if (gamepad1.triangle) {
            buildPaths();
            sleep(40);
            follower.followPath(driveToGoal, 0.4, true);
            sleep(300);
        }
        if (gamepad1.square) {

            turn.setPosition(SampleDegrees/180+0.2);
            sleep(400);
            // ---- new slide movement ----
            double targetTicks = 117 * (distanceForwardBackwardFromGoal - 1.5);
            targetTicks = Math.max(0, Math.min(1980, targetTicks)); // clamp between 0 and 1980
            slide.setTargetPosition((int) Math.round(targetTicks));
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
            // ----------------------------

            sleep(200);
            SampleDegrees                  = 0;
            xDistance                      = 0;
            yDistance                      = 0;
            a                              = 0;
            b                              = 0;
            distanceLeftRightFromGoal      = 0;
            distanceForwardBackwardFromGoal= 0;
        }

        if (gamepad1.circle) {
        claw.setPosition(1);
        sleep(300);
        pivot.setPosition(0);
        sleep(500);
        claw.setPosition(0);
        sleep(500);
        pivot.setPosition(1);

        }

        if (gamepad1.dpad_up) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.DetectorResult dr : result.getDetectorResults()) {
                    String cls = dr.getClassName().toLowerCase();
                    if (!cls.equals("yellow") && !cls.equals("red")) continue;

                    List<List<Double>> corners = dr.getTargetCorners();
                    if (corners == null || corners.size() != 4) continue;

                    // compute distances and angle
                    distanceForwardBackwardFromGoal =
                            Math.tan(Math.toRadians(-dr.getTargetXDegrees())) * 13.0;
                    distanceLeftRightFromGoal =
                            Math.tan(Math.toRadians(dr.getTargetYDegrees())) * 13.0;

                    double topLeftX     = corners.get(0).get(0);
                    double topLeftY     = corners.get(0).get(1);
                    double bottomRightX = corners.get(2).get(0);
                    double bottomRightY = corners.get(2).get(1);

                    xDistance = bottomRightX - topLeftX;
                    yDistance = bottomRightY - topLeftY;

                    a = (((xDistance * -7.0/3.0) + yDistance)) / (-40.0/9.0);
                    b = (((yDistance * -7.0/3.0) + xDistance)) / (-40.0/9.0);
                    SampleDegrees = Math.toDegrees(Math.atan2(a, b));

                    break;  // handle only first valid target
                }
            }
        }

        // always show the last computed values
        telemetry.addData("LRDeltaX", LRDeltaX);
        telemetry.addData("LRDeltaY", LRDeltaY);
        telemetry.addData("X Distance",    xDistance);
        telemetry.addData("Y Distance",    yDistance);
        telemetry.addData("A",             a);
        telemetry.addData("B",             b);
        telemetry.addData("Angle (deg)",  "%.2f", SampleDegrees);
        telemetry.addData("Dist L/R",     "%.2f", distanceLeftRightFromGoal);
        telemetry.addData("Dist F/B",     "%.2f", distanceForwardBackwardFromGoal);
        telemetry.update();

        follower.update();
    }

    @Override public void stop() { }
}
