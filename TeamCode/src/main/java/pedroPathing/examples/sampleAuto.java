package pedroPathing.examples;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

import pedroPathing.RandomClass;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled
@Autonomous(name = "sampleAuto", group = "adon")
public class sampleAuto extends OpMode {
    private Follower follower;
    private Limelight3A limelight;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose          = new Pose(136.5, 40.5, Math.toRadians(270));
    private final Pose outtake            = new Pose(125, 17, Math.toRadians(315));
    private final Pose outtakeINTAKEONE   = new Pose(113.5, 23, Math.toRadians(180));
    private final Pose outtakeINTAKETWO   = new Pose(113, 14.5, Math.toRadians(180));
    private final Pose outtakeINTAKETHREE = new Pose(98, 18.4, Math.toRadians(270));
    private final Pose intakeTwoControl   = new Pose(114, 30, Math.toRadians(180));
    private final Pose sub                = new Pose(84, 45, Math.toRadians(90));
    private final Pose subway             = new Pose(84, 24, Math.toRadians(90));

    private Servo claw, pivot, turn;
    private DcMotor slide;
    private PathChain action1, action2, action3, action4, action5, action6, action7, action8, action9;

    private double SampleDegrees;
    private double xDistance, yDistance, a, b;
    private double distanceLeftRightFromGoal, distanceForwardBackwardFromGoal;
    private double LRDeltaX, LRDeltaY;
    private double backwardsDistance, FBDeltaX, FBDeltaY;
    private PathChain driveToGoal;

    @Override
    public void init() {
        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        claw  = hardwareMap.get(Servo.class, "claw");
        pivot = hardwareMap.get(Servo.class, "pivot");
        turn  = hardwareMap.get(Servo.class, "turn");

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        pathTimer   = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        buildPaths();
    }

    @Override
    public void start() {
        setPathState(0);
        follower.startTeleopDrive();
        follower.setMaxPower(1);
    }

    private void buildPaths() {
        action1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(outtake)))
                .setLinearHeadingInterpolation(startPose.getHeading(), outtake.getHeading())
                .build();

        action2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(outtake), new Point(outtakeINTAKEONE)))
                .setLinearHeadingInterpolation(outtake.getHeading(), outtakeINTAKEONE.getHeading())
                .setZeroPowerAccelerationMultiplier(3)
                .setPathEndTimeoutConstraint(400)
                .build();

        action3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakeINTAKEONE), new Point(intakeTwoControl), new Point(outtake)))
                .setLinearHeadingInterpolation(outtakeINTAKEONE.getHeading(), outtake.getHeading())
                .build();

        action4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtake), new Point(intakeTwoControl), new Point(outtakeINTAKETWO)))
                .setLinearHeadingInterpolation(outtake.getHeading(), outtakeINTAKETWO.getHeading())
                .setPathEndTimeoutConstraint(400)
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        action5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakeINTAKETWO), new Point(intakeTwoControl), new Point(outtake)))
                .setLinearHeadingInterpolation(outtakeINTAKETWO.getHeading(), outtake.getHeading())
                .build();

        action6 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(outtake), new Point(outtakeINTAKETHREE)))
                .setLinearHeadingInterpolation(outtake.getHeading(), outtakeINTAKETHREE.getHeading())
                .setPathEndTimeoutConstraint(200)
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        action7 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakeINTAKETHREE), new Point(intakeTwoControl), new Point(outtake)))
                .setLinearHeadingInterpolation(outtakeINTAKETHREE.getHeading(), outtake.getHeading())
                .build();

        action8 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtake), new Point(subway), new Point(sub)))
                .setLinearHeadingInterpolation(outtake.getHeading(), sub.getHeading())
                .setPathEndTimeoutConstraint(400)
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        action9 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sub), new Point(subway), new Point(outtake)))
                .setLinearHeadingInterpolation(sub.getHeading(), outtake.getHeading())
                .build();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                pivot.setPosition(0.5);
                claw.setPosition(0);
                turn.setPosition(0.5);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(action1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    claw.setPosition(0.6);
                    sleep(100);
                    follower.followPath(action2, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.holdPoint(outtakeINTAKEONE);
                    pivot.setPosition(0);
                    sleep(225);
                    claw.setPosition(0);
                    sleep(350);
                    pivot.setPosition(0.6);
                    follower.followPath(action3, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.holdPoint(outtake);
                    claw.setPosition(0.6);
                    sleep(100);
                    follower.followPath(action4, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.holdPoint(outtakeINTAKETWO);
                    pivot.setPosition(0);
                    sleep(275);
                    claw.setPosition(0);
                    sleep(350);
                    pivot.setPosition(0.6);
                    follower.followPath(action5, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.holdPoint(outtake);
                    claw.setPosition(0.6);
                    sleep(100);
                    follower.followPath(action6, true);
                    turn.setPosition(0);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.holdPoint(outtakeINTAKETHREE);
                    pivot.setPosition(0);
                    sleep(275);
                    claw.setPosition(0);
                    sleep(350);
                    pivot.setPosition(0.6);
                    follower.followPath(action7, true);
                    turn.setPosition(0.5);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.holdPoint(outtake);
                    claw.setPosition(0.6);
                    sleep(100);
                    follower.followPath(action8, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.holdPoint(sub);
                    sleep(500);
                    LLResult res = limelight.getLatestResult();
                    if (res != null && res.isValid()) {
                        telemetry.addData("Target Found", true);
                        for (LLResultTypes.DetectorResult dr : res.getDetectorResults()) {
                            String cls = dr.getClassName().toLowerCase();
                            if (!cls.equals("yellow") && !cls.equals("red")) continue;
                            telemetry.addData("Detected Object", cls);

                            distanceForwardBackwardFromGoal = Math.tan(Math.toRadians(-dr.getTargetXDegrees())) * 13.0;
                            distanceLeftRightFromGoal      = Math.tan(Math.toRadians(dr.getTargetYDegrees()))  * 13.0;

                            List<List<Double>> c = dr.getTargetCorners();
                            double tlx = c.get(0).get(0), tly = c.get(0).get(1);
                            double brx = c.get(2).get(0), bry = c.get(2).get(1);
                            xDistance = brx - tlx;
                            yDistance = bry - tly;
                            a = ((xDistance * -7.0/3.0) + yDistance) / (-40.0/9.0);
                            b = ((yDistance * -7.0/3.0) + xDistance)  / (-40.0/9.0);
                            SampleDegrees = Math.toDegrees(Math.atan2(a, b));
                            break;
                        }

                        double hd = follower.getPose().getHeading();
                        if (distanceLeftRightFromGoal == 0) {
                            LRDeltaX = LRDeltaY = 0;
                        } else {
                            LRDeltaX = (distanceLeftRightFromGoal + 4.5) *
                                    Math.cos(-(Math.PI/2 - hd));
                            LRDeltaY = (distanceLeftRightFromGoal + 4.5) *
                                    Math.sin(-(Math.PI/2 - hd));
                        }

                        int ticks = (int)Math.round(117 * (distanceForwardBackwardFromGoal - 1.5));
                        backwardsDistance = 0;
                        if (ticks < 0) {
                            backwardsDistance = ticks / 130.0;
                            ticks = 0;
                        }

                        FBDeltaX = backwardsDistance * Math.cos(hd);
                        FBDeltaY = backwardsDistance * Math.sin(hd);

                        driveToGoal = follower.pathBuilder()
                                .addPath(new BezierLine(
                                        new Point(follower.getPose()),
                                        new Point(
                                                follower.getPose().getX() + LRDeltaX + FBDeltaX,
                                                follower.getPose().getY() + LRDeltaY + FBDeltaY
                                        )
                                ))
                                .setLinearHeadingInterpolation(
                                        follower.getPose().getHeading(),
                                        follower.getPose().getHeading()
                                )
                                .build();

                        follower.followPath(driveToGoal, 1, false);

                        ticks = Math.min(ticks, 1980);
                        slide.setTargetPosition(ticks);
                        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slide.setPower(1);

                        sleep(800);
                        turn.setPosition(SampleDegrees / 180.0 + 0.2);
                        sleep(400);
                        setPathState(10);
                    } else {
                        telemetry.addData("Target Found", false);
                    }
                    telemetry.update();
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    pivot.setPosition(0);
                    sleep(300);
                    claw.setPosition(0);
                    sleep(300);
                    pivot.setPosition(0.6);
                    sleep(200);
                    slide.setTargetPosition(0);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    sleep(500);
                    setPathState(11);
                }
                break;
            case 11:
                telemetry.addData("Intake complete", true);
                telemetry.update();
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void stop() {
        follower.update();
        RandomClass.autoEnd = follower.getPose();
    }
}
