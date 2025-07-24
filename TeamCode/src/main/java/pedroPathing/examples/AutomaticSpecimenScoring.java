package pedroPathing.examples;

import static android.os.SystemClock.sleep;

import static pedroPathing.RandomClass.autoEnd;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled
@TeleOp(name = "AutomaticSpecimenScoring", group = "adon")

public class AutomaticSpecimenScoring extends OpMode {

    Follower follower;
    private PathChain driveToBasket, driveAdon, driveToHumanPlayer, driveToOrigin, driveToSpec, driveToTest, driveFiveInchesRight, driveFiveInchesLeft;

    private final Pose origin = new Pose(0, 0, Math.toRadians(0));
    private final Pose humanPlayer = new Pose(136, 120, Math.toRadians(0));
    private final Pose specPosition = new Pose(106, 80, Math.toRadians(180));
    private final Pose basket = new Pose(122, 22, Math.toRadians(135));
    private final Pose test = new Pose(130, 84, Math.toRadians(180));
    private final Pose adonStart = new Pose(136.5, 79.5, Math.toRadians(180));
    public static double DISTANCE = 10;


    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(autoEnd);
        DcMotor slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        follower.breakFollowing();
        follower.setMaxPower(1.0);
        follower.startTeleopDrive();
        buildPaths();

    }

    @Override
    public void init_loop() {
    }

    public void buildPaths() {

        driveToSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(specPosition)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), specPosition.getHeading())
                .build();

        driveToHumanPlayer = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(follower.getPose()), new Point(adonStart), new Point(humanPlayer)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), humanPlayer.getHeading())
                .build();

        driveAdon = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(adonStart)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), adonStart.getHeading())
                .build();

        driveToBasket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(basket)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), basket.getHeading())
                .build();

    }

    @Override
    public void loop() {

        double DeltaX = (DISTANCE * (Math.cos((-1 * (((3.14159 / 2) - (follower.getPose().getHeading())))))));
        double DeltaY = (DISTANCE * (Math.sin((-1 * (((3.14159 / 2) - (follower.getPose().getHeading())))))));

        if (!follower.isBusy()) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }

        if (gamepad1.square) {
            sleep(20);
            follower.followPath(driveToHumanPlayer, 0.8, true);

        }

        if (gamepad1.circle) {
            sleep(20);
            follower.followPath(driveToSpec, 0.8, true);

        }

        if (gamepad1.cross) {
            sleep(20);
            follower.followPath(driveAdon, 0.8, true);

        }

        if (gamepad1.triangle) {
            follower.breakFollowing();
            follower.setMaxPower(1.0);
            follower.startTeleopDrive();
        }

        if (gamepad1.dpad_left) {
            sleep(20);
            follower.followPath(driveToBasket, 0.8, true);
        }

        if (gamepad1.right_bumper) {
            Pose righttemp = new Pose((follower.getPose().getX() + DeltaX), (follower.getPose().getY() + DeltaY), follower.getPose().getHeading());
            sleep(20);
            follower.holdPoint(righttemp);
        }

        if (gamepad1.left_bumper) {
            Pose lefttemp = new Pose((follower.getPose().getX() - DeltaX), (follower.getPose().getY() - DeltaY), follower.getPose().getHeading());
            sleep(20);
            follower.holdPoint(lefttemp);
        }


            follower.update();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("deltaX", DeltaX);
            telemetry.addData("deltaY", DeltaY);
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));


            telemetry.update();
        }
    }