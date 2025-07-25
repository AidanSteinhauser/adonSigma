package pedroPathing;

import static android.os.SystemClock.sleep;
import static pedroPathing.RandomClass.autoEnd;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp(name = "Teleop", group = "adon")
public class Teleop extends OpMode {
    private Follower follower;

    private double turnValue = 0.5;
    private double slideticks = 0;
    private double pivotValue = 1;
    private double clawValue = 0.55;
    private double driveSpeedDivisor = 0;
    private double nudgeDistance = 1.5;


    private DcMotor slide = null;
    private Servo claw = null;
    private Servo pivot = null;
    private Servo turn = null;

    private final Pose basketTest = new Pose((118), (26), Math.toRadians(315));
    private final Pose subtemp = new Pose((75), (45), Math.toRadians(90));
    private final Pose subway = new Pose((84), (24), Math.toRadians(90));
    private PathChain driveToSub, driveToBasket;
    /**
     * This method is call once when init is played, it initializes the follower
     **/
    public void buildPaths() {
        driveToSub = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(follower.getPose()), new Point(subway), new Point(subtemp)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), subtemp.getHeading())
                .build();

        driveToBasket = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(follower.getPose()), new Point(subway),  new Point(basketTest)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), basketTest.getHeading())
                .build();

        driveToBasket = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(follower.getPose()), new Point(subway),  new Point(subway),  new Point(subway),  new Point(basketTest)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), basketTest.getHeading())
                .build();

    }

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(autoEnd);
        slide = hardwareMap.get(DcMotor.class, "slide");
        claw = hardwareMap.get(Servo.class, "claw");
        pivot = hardwareMap.get(Servo.class, "pivot");
        turn = hardwareMap.get(Servo.class, "turn");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        buildPaths();

    }

    /**
     * This method is called continuously after Init while waiting to be started.
     **/
    @Override
    public void init_loop() {
        buildPaths();
    }

    /**
     * This method is called once at the start of the OpMode.
     **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /**
     * This is the main loop of the opmode and runs continuously after play
     **/
    @Override
    public void loop() {

        double LRDeltaX = (nudgeDistance * (Math.cos((-1 * (((3.14159 / 2) - (follower.getPose().getHeading())))))));
        double LRDeltaY = (nudgeDistance * (Math.sin((-1 * (((3.14159 / 2) - (follower.getPose().getHeading())))))));

        double FBDeltaX = (nudgeDistance * (Math.cos(follower.getPose().getHeading())));
        double FBDeltaY = (nudgeDistance * (Math.sin(follower.getPose().getHeading())));

        if (!follower.isBusy()) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x*0.5, true);
        }

        if (gamepad1.ps) {
            follower.breakFollowing();
            follower.setMaxPower(1.0);
            follower.startTeleopDrive();
        }

        if (gamepad1.right_trigger > 0.2) {
            slideticks = 100;
            sleep(20);
            slide.setTargetPosition((int) slideticks);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotValue = 1;
            pivot.setPosition(1);
            sleep(200);
            follower.followPath(driveToBasket, 1, true);
        }

        if (gamepad1.left_trigger > 0.2) {
            slideticks = 100;
            sleep(20);
            slide.setTargetPosition((int) slideticks);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotValue = 1;
            pivot.setPosition(1);
            sleep(200);
            follower.followPath(driveToSub, 1, true);
        }

        if (gamepad1.right_bumper) {
            Pose righttemp = new Pose((follower.getPose().getX() + LRDeltaX), (follower.getPose().getY() + LRDeltaY), follower.getPose().getHeading());
            sleep(20);
            follower.setMaxPower(1);
            follower.holdPoint(righttemp);
        }

        if (gamepad1.left_bumper) {
            Pose lefttemp = new Pose((follower.getPose().getX() - LRDeltaX), (follower.getPose().getY() - LRDeltaY), follower.getPose().getHeading());
            sleep(20);
            follower.setMaxPower(1);
            follower.holdPoint(lefttemp);
        }


        if (gamepad1.dpad_up) {
            Pose forwardtemp = new Pose((follower.getPose().getX() + FBDeltaX), (follower.getPose().getY() + FBDeltaY), follower.getPose().getHeading());
            sleep(20);
            follower.setMaxPower(0.7);
            follower.holdPoint(forwardtemp);
        }

        if (gamepad1.dpad_down) {
            Pose backwardtemp = new Pose((follower.getPose().getX() - FBDeltaX), (follower.getPose().getY() - FBDeltaY), follower.getPose().getHeading());
            sleep(20);
            follower.setMaxPower(0.7);
            follower.holdPoint(backwardtemp);
        }












        if (gamepad1.cross) {
            slideticks = 20;
        }

        if (gamepad1.triangle) {
            slideticks = 2000;
        }


        if (gamepad1.dpad_left) {
            if (turnValue < 0.83) {
                turnValue = turnValue + 0.02;
            }
        }

        if (gamepad1.dpad_right) {
            if (turnValue > 0.02) {
                turnValue = turnValue - 0.02;
            }
        }




        if (gamepad1.share) {
            if (1 == pivot.getPosition()) {
                pivotValue = 0;
            }
        }
        if (gamepad1.share) {
            if (0 == pivot.getPosition()) {
                pivotValue = 1;
            }
        }
        if (!gamepad1.share) {
            pivot.setPosition(pivotValue);
        }

        if (gamepad1.options) {
            if (0.55 == claw.getPosition()) {
                clawValue = 0;
            }
        }

        if (gamepad1.options) {
            if (0 == claw.getPosition()) {
                    clawValue = 0.55;
            }
        }
        if (!gamepad1.options) {
            claw.setPosition(clawValue);
        }



            if (gamepad1.circle && slideticks < 1975) {
                slideticks += 40;
            }
            if (gamepad1.square && slideticks > 50) {
                slideticks += -40;
            }

            turn.setPosition(turnValue);
            slide.setPower(1);
            slide.setTargetPosition((int) slideticks);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("pivot position", pivot.getPosition());
            telemetry.addData("claw position", claw.getPosition());
            telemetry.addData("turn var", turnValue);
            telemetry.addData("turn position", turn.getPosition());
            telemetry.addData("slide ticks", slide.getCurrentPosition());
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

            follower.update();
            telemetry.update();

        }

        /** We do not use this because everything automatically should disable **/
        @Override
        public void stop() {
        }
    }