package pedroPathing;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.RandomClass;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "PedroLevelThreeNationalNightOut", group = "Adon")
public class PedroLevelThreeNationalNightOut extends OpMode {

    private Follower follower;
    private final Pose startPose = new Pose(55.25, 17, Math.toRadians(90));
    private final Pose puff = new Pose(50.5, 48, Math.toRadians(90));
    private final Pose pizza = new Pose(59.25, 36, Math.toRadians(0));

    private PathChain pasta, veggiewrap;
    private Servo claw;
    private int pathState;

    private Timer pathTimer;
    private Timer opmodeTimer;

    public void buildPaths() {
        pasta = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(puff)))
                .setLinearHeadingInterpolation(startPose.getHeading(), puff.getHeading())
                .setPathEndTimeoutConstraint(400)
                .build();

        veggiewrap = follower.pathBuilder()
                .addPath(new BezierLine(new Point(puff), new Point(pizza)))
                .setLinearHeadingInterpolation(puff.getHeading(), pizza.getHeading())
                .build();
    }

    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(pasta, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.holdPoint(puff);
                    claw.setPosition(0);
                    sleep(800);
                    follower.followPath(veggiewrap, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.holdPoint(pizza);
                    claw.setPosition(1);
                    sleep(800);
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        claw = hardwareMap.get(Servo.class, "claw");
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.update();
        RandomClass.autoEnd = follower.getPose();
    }
}
