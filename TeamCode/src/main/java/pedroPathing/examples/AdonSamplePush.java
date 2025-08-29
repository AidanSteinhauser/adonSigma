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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.RandomClass;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Disabled
@Autonomous(name = "AdonSamplePush", group = "adon")
public class AdonSamplePush extends OpMode {
    private Follower follower;
    private Telemetry telemetryA;
    private Timer pathTimer;
    private Timer sillyTimer;
    private Timer actionTimer;
    private Timer opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(136.5, 79.5, Math.toRadians(180));
    private final Pose firstPose = new Pose(120, 80, Math.toRadians(180));
    private final Pose secondPoseControl = new Pose(110, 80, Math.toRadians(0));
    private final Pose secondPose = new Pose(107, 100, Math.toRadians(0));
    private final Pose thirdPose = new Pose(130, 100, Math.toRadians(0));
    private final Pose fourthPoseControl = new Pose(107, 100, Math.toRadians(0));
    private final Pose fourthPose = new Pose(107, 120, Math.toRadians(0));
    private final Pose fifthPose = new Pose(130, 120, Math.toRadians(0));
    private final Pose eighthPose = new Pose(136, 125, Math.toRadians(180));

    private PathChain action1, action2, action3, action4, action5, action6, action7, action8, action9;
    private DcMotor slide = null;

    public void buildPaths() {
        action1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(firstPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstPose.getHeading())
                .build();

        action2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstPose), new Point(secondPoseControl), new Point(secondPose)))
                .setLinearHeadingInterpolation(firstPose.getHeading(), secondPose.getHeading())
                .build();

        action3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondPose), new Point(thirdPose)))
                .setLinearHeadingInterpolation(secondPose.getHeading(), thirdPose.getHeading())
                .build();

        action4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(thirdPose), new Point(fourthPoseControl), new Point(fourthPose)))
                .setLinearHeadingInterpolation(thirdPose.getHeading(), fourthPose.getHeading())
                .build();

        action5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourthPose), new Point(fifthPose)))
                .setLinearHeadingInterpolation(fourthPose.getHeading(), fifthPose.getHeading())
                .build();

        action6 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(fifthPose), new Point(fourthPose), new Point(firstPose)))
                .setLinearHeadingInterpolation(fifthPose.getHeading(), firstPose.getHeading())
                .build();

        action7 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstPose), new Point(startPose)))
                .setLinearHeadingInterpolation(firstPose.getHeading(), startPose.getHeading())
                .build();

        action8 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(eighthPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), eighthPose.getHeading())
                .build();

        action9 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eighthPose), new Point(startPose)))
                .setLinearHeadingInterpolation(eighthPose.getHeading(), startPose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
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
                    follower.holdPoint(secondPose);
                    sleep(450);
                    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slide.setDirection(DcMotorSimple.Direction.REVERSE);
                    slide.setPower(0.6);
                    slide.setTargetPosition(500);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(900);
                    slide.setDirection(DcMotorSimple.Direction.REVERSE);
                    slide.setPower(0.6);
                    slide.setTargetPosition(0);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(900);
                    follower.followPath(action2, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(action3, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(action4, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(action5, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(action6, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(action7, true);
                    setPathState(8);
                }
            case 8:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions
     * It will also reset the timers of the individual switches
     **/
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

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
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