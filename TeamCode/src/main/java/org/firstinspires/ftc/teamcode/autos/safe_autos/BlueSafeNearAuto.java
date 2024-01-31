package org.firstinspires.ftc.teamcode.autos.safe_autos;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ArmV2;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.Globals;
import org.firstinspires.ftc.teamcode.vision.Location;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Config
@Autonomous(name = "BLUENearAuto_Safe", group = "1Safe_Autos")
//@Disabled
public class BlueSafeNearAuto extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    ArmV2 arm = null;
    Hanger hanger = null;
    Intake intake = null;


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    public static int val=0;
    double x;
    double y;
    String propPosition = " ";

    public static double
            lifter_posL = 0, lifter_posR = 0, error_lifter, error_diff, error_int, error_lifterR, error_diffR, error_intR, errorprev, errorprevR, output_lifter, output_lifterR, output_power, target, dropVal;
    public static double armServoOnePos = 0.92, armServoOneUP = 0.7, armServoOneOut = 0.49;
    public static double kp = 4, ki, kd = 1.7;

    public static double yellowDiff = 1.5;
    private PropPipeline propPipeline;
    private VisionPortal portal;
    private Location randomization;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        slider = new Slider(hardwareMap, telemetry);
        arm = new ArmV2(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.CLOSE;

        Pose2d startPose=new Pose2d(14, 62, -Math.PI);
        drive.setPoseEstimate(startPose);

        TrajectorySequence AutoTrajectoryRight = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(this::telem)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                //backdrop
                .lineToConstantHeading(new Vector2d(17.5 , 30))

                .addTemporalMarker(()->{Intake.CrankPosition(0.35);arm.setArmPos(armServoOneUP, 0.16);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Intake.CrankPosition(0.42);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .lineToConstantHeading(new Vector2d(50.5 - yellowDiff,27))

                .waitSeconds(0.4)
                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.65);})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.4)//0.55
                .addTemporalMarker(this::telem)
                .resetConstraints()
                .addTemporalMarker(() -> {arm.setArmPos(0.50, 0.68);})
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {arm.setArmPos(armServoOneUP, 0.16);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(armServoOnePos, 0.16);})
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(51, 60))
                .setReversed(false)
                .waitSeconds(30)
                .build();

        TrajectorySequence AutoTrajectoryCenter = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(this::telem)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                //backdrop
                .lineToConstantHeading(new Vector2d(25 , 24))
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.16);})
                .waitSeconds(0.5)
                .addTemporalMarker(this::telem)
                .addTemporalMarker(()->{Intake.CrankPosition(0.45);})
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .splineToConstantHeading(new Vector2d(51 - yellowDiff,35.5), 0)

                .waitSeconds(0.3)
                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.67);})
                .waitSeconds(1)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(1)//0.55
                .addTemporalMarker(this::telem)
                .resetConstraints()
                .addTemporalMarker(() -> {arm.setArmPos(0.5, 0.68);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(armServoOneUP, 0.16);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(armServoOnePos, 0.16);})
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(50, 60))
                .setReversed(false)
                .waitSeconds(30)
                .build();

        TrajectorySequence AutoTrajectoryLeft = drive.trajectorySequenceBuilder(startPose)

                .addTemporalMarker(this::telem)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                //backdrop
                .lineToConstantHeading(new Vector2d(33 , 28))

                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.16);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.CrankPosition(0.65);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.16);})
                .waitSeconds(0.5)
                .addTemporalMarker(this::telem)

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
//                .splineToConstantHeading(new Vector2d(51 - yellowDiff,43), 0)
                .lineToConstantHeading(new Vector2d(51 - yellowDiff,43))

                .waitSeconds(0.3)
                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.68);})
                .waitSeconds(1)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(1)//0.55
                .addTemporalMarker(this::telem)
                .resetConstraints()
                .addTemporalMarker(() -> {arm.setArmPos(0.5, 0.68);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(armServoOneUP, 0.16);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(armServoOnePos, 0.16);})
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(50.5, 60))
                .setReversed(false)
                .waitSeconds(30)
                .build();

        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(propPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        while (getCameraState() != VisionPortal.CameraState.STREAMING && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("initializing... please wait");
            telemetry.update();
        }

        while (opModeInInit()){
            slider.extendToHome();
            ArmV2.SetArmPosition(armServoOnePos, 0.16);
            Intake.SetArmPosition(0.5,0.66);
            Intake.IntakePixel(0.8);
            ArmV2.DropPixel(0.5);
            Intake.CrankPosition(0.69);
            ArmV2.SliderLink(0.95);

            telemetry.addLine("ready");
            try{
                propPosition = propPipeline.getLocation().toString();
            }catch (Exception e){
                propPosition="RIGHT";
            }
            telemetry.addData("position", propPipeline.getLocation());
            telemetry.addData("marker",propPosition);
            telemetry.update();
        }
        portal.close();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.b || propPosition == "RIGHT"){
                drive.followTrajectorySequence(AutoTrajectoryRight);
                propPosition = " ";
            }
            if (gamepad1.y || propPosition == "CENTER"){
                drive.followTrajectorySequence(AutoTrajectoryCenter);
                propPosition = " ";
            }
            if (gamepad1.x || propPosition == "LEFT"){
                drive.followTrajectorySequence(AutoTrajectoryLeft);
                propPosition = " ";
            }
            telemetry.addData("LeftFrontCurrent", drive.getMotorCurrent().get(0));
            telemetry.addData("RightFrontCurrent", drive.getMotorCurrent().get(1));
            telemetry.addData("LeftRearCurrent", drive.getMotorCurrent().get(2));
            telemetry.addData("RightRearCurrent", drive.getMotorCurrent().get(3));
            telemetry.addData("position",propPosition);
            sleep(500);
            drive.update();
            telemetry.update();
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    public VisionPortal.CameraState getCameraState() {
        if (portal != null) return portal.getCameraState();
        return null;
    }
    public void telem(){
        telemetry.addData("LeftFrontCurrent", drive.getMotorCurrent().get(0));
        telemetry.addData("RightFrontCurrent", drive.getMotorCurrent().get(1));
        telemetry.addData("LeftRearCurrent", drive.getMotorCurrent().get(2));
        telemetry.addData("RightRearCurrent", drive.getMotorCurrent().get(3));
        telemetry.update();
    }
    public double lifter_pid(double kp_lifter, double ki_lifter, double kd_lifter, int target)
    {
        lifter_posL = Slider.sliderMotorOne.getCurrentPosition();
        lifter_posR = Slider.sliderMotorTwo.getCurrentPosition();
        error_lifter = target - lifter_posL;
        error_diff = error_lifter - errorprev;
        error_int = error_lifter + errorprev;
        output_lifter = kp_lifter*error_lifter + kd_lifter*error_diff +ki_lifter*error_int;
        error_lifterR = target - lifter_posR;
        error_diffR = error_lifterR - errorprevR;
        error_intR = error_lifterR + errorprevR;
        output_lifterR = kp_lifter*error_lifterR + kd_lifter*error_diffR +ki_lifter*error_intR;
        errorprev = error_lifter;
        errorprevR = error_lifterR;
        return Math.abs(output_lifter);
    }
}
