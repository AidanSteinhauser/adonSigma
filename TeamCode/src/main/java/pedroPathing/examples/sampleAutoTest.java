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

import pedroPathing.RandomClass;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled
@Autonomous(name = "sampleAutoTest", group = "adon")
public class sampleAutoTest extends OpMode {
    private Follower follower;
    private Telemetry telemetryA;
    private Timer pathTimer;
    private Timer sillyTimer;
    private Timer actionTimer;
    private Timer opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(136.5, 40.5, Math.toRadians(270));
    private final Pose intakeOne = new Pose(113, 24.75, Math.toRadians(180));
    private final Pose intakeTwo = new Pose(113, 16.5, Math.toRadians(180));
    private final Pose intakeThree = new Pose(98, 19.25, Math.toRadians(270));
    private final Pose intakeTwoControl = new Pose(114, 28, Math.toRadians(180));
    private final Pose outtake = new Pose(127, 17, Math.toRadians(315));
    private final Pose subtemp = new Pose((84), (33), Math.toRadians(90));
    private final Pose subcontrol = new Pose((84), (24), Math.toRadians(90));
    private Servo claw  = null;
    private Servo pivot  = null;
    private Servo turn  = null;
    private PathChain action0,actionbruh, yovoyahSAMPLES, action1, action2, action3, action4, action5, action6, action7, action8, action9;
    private DcMotor slide = null;

    public void buildPaths() {
        action0 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(outtake)))
                .setLinearHeadingInterpolation(startPose.getHeading(), outtake.getHeading())
                .build();

        action1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(outtake), new Point(intakeOne)))
                .setLinearHeadingInterpolation(outtake.getHeading(), intakeOne.getHeading())
                .build();

        action2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(intakeOne), new Point(intakeTwoControl), new Point(outtake)))
                .setLinearHeadingInterpolation(intakeOne.getHeading(), outtake.getHeading())
                .build();

        action3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtake), new Point(intakeTwoControl), new Point(intakeTwo)))
                .setLinearHeadingInterpolation(outtake.getHeading(), intakeTwo.getHeading())
                .build();
        actionbruh = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(intakeTwo), new Point(intakeTwoControl), new Point(outtake)))
                .setLinearHeadingInterpolation(intakeTwo.getHeading(), outtake.getHeading())
                .build();


        action4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(outtake), new Point(intakeThree)))
                .setLinearHeadingInterpolation(outtake.getHeading(), intakeThree.getHeading())
                .build();

        action5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(intakeThree), new Point(intakeTwoControl), new Point(outtake)))
                .setLinearHeadingInterpolation(intakeThree.getHeading(), outtake.getHeading())
                .build();

        yovoyahSAMPLES = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(outtake), new Point(subcontrol), new Point(subtemp)))
                .setLinearHeadingInterpolation(outtake.getHeading(), subtemp.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                pivot.setPosition(0.3);
                claw.setPosition(0);
                turn.setPosition(0.5);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(action0, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.holdPoint(outtake);
                    claw.setPosition(0.6);
                    sleep(100);
                    follower.followPath(action1, true);
                    claw.setPosition(0.7);
                    pivot.setPosition(0.5);
                    sleep(150);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.holdPoint(intakeOne);
                    sleep(150);
                    pivot.setPosition(0);
                    sleep(200);
                    claw.setPosition(0);
                    sleep(300);
                    follower.followPath(action2, true);
                    sleep(50);
                    pivot.setPosition(0.3);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.holdPoint(outtake);
                    pivot.setPosition(0);
                    sleep(50);
                    claw.setPosition(0.6);
                    sleep(200);
                    pivot.setPosition(0.3);
                    sleep(300);
                    follower.followPath(action3, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.holdPoint(intakeTwo);
                    sleep(150);
                    pivot.setPosition(0);
                    sleep(250);
                    claw.setPosition(0);
                    sleep(350);
                    follower.followPath(actionbruh, true);
                    sleep(50);
                    pivot.setPosition(0.3);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.holdPoint(outtake);
                    pivot.setPosition(0);
                    sleep(50);
                    claw.setPosition(0.6);
                    sleep(200);
                    pivot.setPosition(0.3);
                    sleep(300);
                    follower.followPath(action4, true);
                    turn.setPosition(0);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.holdPoint(intakeThree);
                    sleep(150);
                    pivot.setPosition(0);
                    sleep(250);
                    claw.setPosition(0);
                    sleep(350);
                    follower.followPath(action5, true);
                    sleep(50);
                    pivot.setPosition(0.3);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.holdPoint(outtake);
                    pivot.setPosition(0);
                    sleep(50);
                    claw.setPosition(0.6);
                    sleep(200);
                    pivot.setPosition(0.3);
                    sleep(200);
                    follower.setMaxPower(1);
                    follower.followPath(yovoyahSAMPLES, true);
                    pivot.setPosition(1);
                    claw.setPosition(1);
                    turn.setPosition(0.5);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.holdPoint(subtemp);
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
        claw = hardwareMap.get(Servo .class, "claw");
        pivot = hardwareMap.get(Servo.class, "pivot");
        turn = hardwareMap.get(Servo.class, "turn");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

