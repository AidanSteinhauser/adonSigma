package pedroPathing.examples;


import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.RandomClass;
@Disabled
@Autonomous(name = "fiveSpec", group = "adon")
public class fiveSpec extends OpMode {
    private Follower follower;
    private Telemetry telemetryA;
    private Timer pathTimer;
    private Timer sillyTimer;
    private Timer actionTimer;
    private Timer opmodeTimer;
    private int pathState;
    private final Pose FiveSpecstartPose = new Pose(133, 79.5, Math.toRadians(180));
    private final Pose outtakePose = new Pose(106, 77, Math.toRadians(180));
    private final Pose prePushPoseOne = new Pose(80, 120, Math.toRadians(0));
    private final Pose prePushPoseOneControl = new Pose(141, 128, Math.toRadians(0));
    private final Pose prePushPoseOneControlTuah = new Pose(55, 101, Math.toRadians(0));
    private final Pose PostPushOne = new Pose(125, 120, Math.toRadians(0));
    private final Pose PostPushTwo = new Pose(128, 130, Math.toRadians(0));
    private final Pose prePushPoseTwo = new Pose(80, 130, Math.toRadians(0));
    private final Pose prePushPoseTwoControl = new Pose(80, 115, Math.toRadians(0));
    private final Pose intoutc1 = new Pose(125, 130, Math.toRadians(0));
    private final Pose intoutc2 = new Pose(125, 77, Math.toRadians(0));

    private Servo claw = null;
    private Servo pivot = null;
    private Servo turn = null;
    private PathChain action0, action1, action2, action3, action4, action5, action6, action7, action8, action9, action10, action11, action12;
    private DcMotor slide = null;

    public void buildPaths() {
        action1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(FiveSpecstartPose), new Point(outtakePose)))
                .setLinearHeadingInterpolation(FiveSpecstartPose.getHeading(), outtakePose.getHeading())
                .build();

        action2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtakePose), new Point(prePushPoseOneControl), new Point(prePushPoseOneControlTuah), new Point(prePushPoseOne)))
                .setLinearHeadingInterpolation(outtakePose.getHeading(), prePushPoseOne.getHeading())
                .setPathEndTimeoutConstraint(50)
                .setZeroPowerAccelerationMultiplier(7)
                .addPath(new BezierLine(new Point(prePushPoseOne), new Point(PostPushOne)))
                .setLinearHeadingInterpolation(prePushPoseOne.getHeading(), PostPushOne.getHeading())
                .setPathEndTimeoutConstraint(50)
                .setZeroPowerAccelerationMultiplier(7)
                .addPath(new BezierCurve(new Point(PostPushOne), new Point(prePushPoseTwoControl), new Point(prePushPoseTwo)))
                .setLinearHeadingInterpolation(PostPushOne.getHeading(), prePushPoseTwo.getHeading())
                .setPathEndTimeoutConstraint(50)
                .setZeroPowerAccelerationMultiplier(7)
                .addPath(new BezierLine(new Point(prePushPoseTwo), new Point(PostPushTwo)))
                .setLinearHeadingInterpolation(prePushPoseTwo.getHeading(), PostPushTwo.getHeading())
                .setPathEndTimeoutConstraint(300)
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(PostPushTwo), new Point(intoutc1), new Point(intoutc2), new Point(outtakePose)))
                .setLinearHeadingInterpolation(PostPushTwo.getHeading(), outtakePose.getHeading())
                .setPathEndTimeoutConstraint(300)
                .setZeroPowerAccelerationMultiplier(3)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                pivot.setPosition(1);
                claw.setPosition(0);
                turn.setPosition(0.5);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(action1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.holdPoint(outtakePose);
                    claw.setPosition(1);
                    sleep(500);
                    follower.followPath(action2, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.holdPoint(PostPushTwo);
                    sleep(700);
                    claw.setPosition(0);
                    sleep(700);
                    follower.followPath(action3, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.holdPoint(outtakePose);
                    claw.setPosition(1);
                    sleep(500);
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        slide = hardwareMap.get(DcMotor.class, "slide");
        claw = hardwareMap.get(Servo .class, "claw");
        pivot = hardwareMap.get(Servo.class, "pivot");
        turn = hardwareMap.get(Servo.class, "turn");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(FiveSpecstartPose);
        buildPaths();
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        follower.update();
        RandomClass.autoEnd = follower.getPose();
    }
}


