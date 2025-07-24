package pedroPathing.examples;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled
@TeleOp(name = "LimelightTurningToSample", group = "adon")
public class LimelightTurningToSample extends OpMode {

    double limelightMountAngleDegrees = 0;
    double limelightLensHeightInches = 4.25;
    double goalHeightInches = 1;
    private Limelight3A limelight;
    private Follower follower;
    private final Pose LimelightTestingStartingPose = new Pose(0, 0, Math.toRadians(180));
    private DcMotor slide = null;
    private Servo claw = null;
    private Servo pivot = null;
    private Servo turn = null;

    double save = 0;
    double MoveXBot = 0;
    double MoveYBot = 0;
    double slideextend = 0;
    boolean dpadLeftPreviouslyPressed = false;



    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(LimelightTestingStartingPose);
        slide = hardwareMap.get(DcMotor.class, "slide");
        claw = hardwareMap.get(Servo.class, "claw");
        pivot = hardwareMap.get(Servo.class, "pivot");
        turn = hardwareMap.get(Servo.class, "turn");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
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
        pivot.setPosition(1);
        claw.setPosition(1);
        turn.setPosition(0);
        follower.startTeleopDrive();
    }

    /**
     * This is the main loop of the opmode and runs continuously after play
     **/
    @Override
    public void loop() {
        limelight.start();
        LLResult result = limelight.getLatestResult();

        if (!follower.isBusy()) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x*0.5, true);
        }

        if (gamepad1.ps) {
            follower.breakFollowing();
            follower.setMaxPower(1.0);
            follower.startTeleopDrive();
        }

        if (result != null) {
            if (result.isValid()) {
                double angleToGoalDegrees = limelightMountAngleDegrees + result.getTy();
                double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(Math.toRadians(angleToGoalDegrees)) + 0.75;
                double distanceMoveRight = (((int) distanceFromLimelightToGoalInches) * Math.sin(Math.toRadians((int) result.getTx())))+3.75;
                double distanceMoveForward = (((int) distanceFromLimelightToGoalInches) * Math.cos(Math.toRadians((int) result.getTx())))-10;

                if ((gamepad1.dpad_left) && !dpadLeftPreviouslyPressed)
                {
                     slideextend = 117*distanceMoveForward;
                     MoveXBot= (distanceMoveRight * (Math.cos((-1 * (((3.14159 / 2) - (follower.getPose().getHeading())))))));
                     MoveYBot = (distanceMoveRight * (Math.sin((-1 * (((3.14159 / 2) - (follower.getPose().getHeading())))))));
                     dpadLeftPreviouslyPressed = true;
                    if (MoveYBot < 30 & MoveXBot < 30 ) {
                        Pose moveforsample = new Pose((follower.getPose().getX() + MoveXBot), (follower.getPose().getY() + MoveYBot), follower.getPose().getHeading());
                        sleep(20);
                        follower.holdPoint(moveforsample);
                        if (slideextend > -1 & slideextend < 1980) {
                            slide.setDirection(DcMotorSimple.Direction.REVERSE);
                            slide.setTargetPosition((int) slideextend);
                            slide.setPower(1);
                            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }
                    }
                }
                if (!gamepad1.dpad_left) {
                    dpadLeftPreviouslyPressed = false;
                }



                telemetry.addData("distance", ((int) distanceFromLimelightToGoalInches));
                telemetry.addData("Distance move Right", (int) distanceMoveRight);
                telemetry.addData("Distance move Up", (int) distanceMoveForward);
                telemetry.addData("Ty", (int) result.getTy());
                telemetry.addData("Tx", (int) result.getTx());
                telemetry.addData("getBotposeAvgArea?", (result.getBotposeAvgArea()));

                telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
            }
        }

        if (gamepad1.dpad_up) {
            claw.setPosition(1);
            sleep(500);
            pivot.setPosition(1);
            sleep(500);
            turn.setPosition(0);
            sleep(500);
            slide.setDirection(DcMotorSimple.Direction.REVERSE);
            slide.setTargetPosition(0);
            slide.setPower(0.5);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);

        }

        if (gamepad1.dpad_down) {
            pivot.setPosition(0);
            sleep(300);
            claw.setPosition(0);
            sleep(300);
        }

        telemetry.addData("MoveXBot", MoveXBot);
        telemetry.addData("MoveYBot", MoveYBot);
        telemetry.addData("slideextend",slideextend);
        telemetry.addData("islimelightconnected", limelight.isConnected());
        telemetry.addData("islimelightrunning", limelight.isRunning());
        follower.update();
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}