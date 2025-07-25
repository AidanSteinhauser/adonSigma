package pedroPathing;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "PedroLevelTwoNationalNightOut", group = "Adon")
public class PedroLevelTwoNationalNightOut extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(55.25, 17, Math.toRadians(90));
    private final Pose pizza = new Pose(68.75, 60, Math.toRadians(0));
    private PathChain pasta;


    private int pathState = 0;
    private Servo claw;

    private void setPathState(int newState) {
        pathState = newState;
    }

    /**
     * This method is call once when init is played, it initializes the follower
     **/
    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class, "claw");
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /**
     * This method is called continuously after Init while waiting to be started.
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     **/
    @Override
    public void start() {
    }

    public void buildPaths() {
        pasta = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(pizza)))
                .setLinearHeadingInterpolation(startPose.getHeading(), pizza.getHeading())
                .build();
    }


    /**
     * This is the main loop of the opmode and runs continuously after play
     **/
    @Override
    public void loop() {
        buildPaths();

        follower.update();
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();


        switch (pathState) {
            case 0:
                if (gamepad1.cross) {
                    follower.setMaxPower(1);
                    follower.followPath(pasta, true);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.holdPoint(pizza);
                    claw.setPosition(1);
                    sleep(800);
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * We do not use this because everything automatically should disable
     **/
    @Override
    public void stop() {
    }
}