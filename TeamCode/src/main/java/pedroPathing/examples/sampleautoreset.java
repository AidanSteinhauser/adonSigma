package pedroPathing.examples;

import static android.os.SystemClock.sleep;
import static pedroPathing.RandomClass.autoEnd;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
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
@TeleOp(name = "sampleautoreset", group = "adon")

public class sampleautoreset extends OpMode {

    Follower follower;
    private PathChain startDrive, driveAdon, driveToHumanPlayer, driveToOrigin, driveToSpec, driveToTest, driveFiveInchesRight, driveFiveInchesLeft;

    private final Pose startPose = new Pose(136.5, 40.5, Math.toRadians(270));
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

        startDrive = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(startPose)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), startPose.getHeading())
                .build();

    }

    @Override
    public void loop() {

        double DeltaX = (DISTANCE * (Math.cos((-1 * (((3.14159 / 2) - (follower.getPose().getHeading())))))));
        double DeltaY = (DISTANCE * (Math.sin((-1 * (((3.14159 / 2) - (follower.getPose().getHeading())))))));

        if (!follower.isBusy()) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }

        if (gamepad1.triangle) {
            follower.breakFollowing();
            follower.setMaxPower(1.0);
            follower.startTeleopDrive();
        }

        if (gamepad1.dpad_left) {
            sleep(20);
            follower.followPath(startDrive, 0.8, true);
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