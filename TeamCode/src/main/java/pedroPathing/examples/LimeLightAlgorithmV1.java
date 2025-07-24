package pedroPathing.examples;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled
@TeleOp(name = "LimeLightAlgorithmV1", group = "adon")
public class LimeLightAlgorithmV1 extends OpMode {
    private Follower follower;
    private PathChain action;
    double DeltaAngleLL = Math.toRadians(19.65);
    double DeltaR = 22.3;
    double SampleAngle = 135;



    double DeltaXLimelight = -3.75;
    double DeltaYLimelight = 12;
    private DcMotor slide = null;
    private Servo claw  = null;
    private Servo pivot  = null;
    private Servo turn  = null;
    double ticks = 0;
    double ExtraMoveTowardsForTicks = 0;
    double ExtraMoveBotX = 0;
    double ExtraMoveBotY = 0;
    Pose testing = new Pose(0, 0, 0);

    @Override
    public void init() {
        slide = hardwareMap.get(DcMotor.class, "slide");
        claw = hardwareMap.get(Servo.class, "claw");
        pivot = hardwareMap.get(Servo.class, "pivot");
        turn = hardwareMap.get(Servo.class, "turn");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(testing);
        claw.setPosition(1);
        pivot.setPosition(1);
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        if (!follower.isBusy()) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }
        if (gamepad1.triangle) {
            follower.breakFollowing();
            follower.setMaxPower(1.0);
            follower.startTeleopDrive();
        }
        follower.update();

        double DeltaLRSample = (Math.sin(DeltaAngleLL) * DeltaR) - DeltaXLimelight;
        double DeltaFBSample = (Math.cos(DeltaAngleLL) * DeltaR) - DeltaYLimelight;
        double MoveXBot = -DeltaLRSample * Math.sin(follower.getPose().getHeading());
        double MoveYBot = -DeltaLRSample * Math.cos(follower.getPose().getHeading());
        double ExtraMoveXBot = ExtraMoveTowardsForTicks*Math.cos(follower.getPose().getHeading());
        double ExtraMoveYBot = ExtraMoveTowardsForTicks*Math.sin(follower.getPose().getHeading());

        if (DeltaFBSample < 17) {
            ticks = (117 * DeltaFBSample);
        }
        else {
            ticks = 1989;
            ExtraMoveTowardsForTicks = DeltaFBSample - 17;
        }

        if (DeltaFBSample < 0) {
            ticks = 0;
            ExtraMoveTowardsForTicks = DeltaFBSample;
        }

        if (gamepad1.dpad_left) {
            Pose turnTemps = new Pose((follower.getPose().getX()+MoveXBot+ExtraMoveXBot), (follower.getPose().getY()+MoveYBot+ExtraMoveYBot), follower.getPose().getHeading());
            sleep(20);
            follower.holdPoint(turnTemps);
            sleep(500);
        }

        if (gamepad1.dpad_right) {
            slide.setDirection(DcMotorSimple.Direction.REVERSE);
            slide.setPower(1);
            slide.setTargetPosition((int) ticks);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
            turn.setPosition((SampleAngle*SampleAngle*(-0.00000925))-0.003055*SampleAngle+0.85);
            sleep(1000);
            pivot.setPosition(0);
            sleep(600);
            claw.setPosition(0);
            sleep(500);
            pivot.setPosition(1);
            sleep(500);
        }


        telemetry.addData("MoveXBot", (MoveXBot));
        telemetry.addData("MoveYBot", (MoveYBot));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("DeltaAngleLL", DeltaAngleLL);
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("DeltaLRSample", (DeltaLRSample));
        telemetry.addData("DeltaFBSample", (DeltaFBSample));
        telemetry.addData("DeltaXLimelight", (DeltaXLimelight));
        telemetry.addData("DeltaYLimelight", (DeltaYLimelight));
        telemetry.addData("ExtraMoveXBot", (ExtraMoveXBot));
        telemetry.addData("ExtraMoveYBot", (ExtraMoveYBot));
        telemetry.addData("ticks", (ticks));
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}