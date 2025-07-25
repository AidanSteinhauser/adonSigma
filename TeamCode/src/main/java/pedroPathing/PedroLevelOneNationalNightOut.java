package pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "PedroLevelOneNationalNightOut", group = "Adon")
public class PedroLevelOneNationalNightOut extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(55.25,17,Math.toRadians(90));
    private final Pose pizza = new Pose(31.5,83,Math.toRadians(90));
    private final Pose controlPose1 = new Pose(62.5,158,Math.toRadians(90));
    private final Pose controlPose2 = new Pose(-82.5,-22,Math.toRadians(90));
    private final Pose controlPose3 = new Pose(117.5,18,Math.toRadians(90));
    private PathChain pasta;


    private int pathState = 0;

    private void setPathState(int newState) {
        pathState = newState;
    }

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
    }
    public void buildPaths() {
        pasta = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(controlPose1),  new Point(controlPose2),  new Point(controlPose3),  new Point(pizza)))
                        .setLinearHeadingInterpolation(startPose.getHeading(),pizza.getHeading())
                        .build();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        if (gamepad1.cross) {
            follower.followPath(pasta, 1, true);
        }


        follower.update();
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}




