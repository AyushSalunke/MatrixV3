package org.firstinspires.ftc.teamcode.autos.cycle_autos;

import android.util.Size;

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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "REDNearAuto_2Cycle", group = "2Cycle_Autos")
@Disabled
public class RedNearAuto extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    ArmV2 arm = null;
    Hanger hanger = null;
    Intake intake = null;
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "blackbox.tflite";
    private static final String[] LABELS = {
            "beacon"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    public static int val=0;
    double x;
    double y;
    String propPosition = " ";

    public static double
            lifter_posL = 0, lifter_posR = 0, error_lifter, error_diff, error_int, error_lifterR, error_diffR, error_intR, errorprev, errorprevR, output_lifter, output_lifterR, output_power, target, dropVal;

    public static double kp = 4, ki, kd = 1.7;

    public static double stackDiff = 0;

    public static Pose2d PurpleLeftPos, YellowLeftPos, StackLeftPos = new Pose2d(-53.5 , -10, -Math.PI); //-51,-12
    public static Vector2d PurpleLeft = new Vector2d(16 , -29), YellowLeft = new Vector2d(50,-26), StackLeft; //25

    public static Pose2d PurpleCenterPos, YellowCenterPos, StackCenterPos;
    public static Vector2d PurpleCenter = new Vector2d(25 , -21), YellowCenter = new Vector2d(53,-35), StackCenter;

    public static Pose2d PurpleRightPos, YellowRightPos, StackRightPos;
    public static Vector2d PurpleRight = new Vector2d(35 , -30), YellowRight = new Vector2d(52,-41), StackRight;

    public static double wristPlay1 = 0.00, wristPlay2 = 0.00;

    enum AutoTrajectoryLeft {
        Start,
        AutoTrajectoryLeft_Purple_Yellow,
        CenterPathPicking,
        CenterPathPlacing,
        WhiteDrop_Left,
        CenterPathPicking2,
        CenterPathPlacing2,
        WhiteDrop2_Left,
        ParkingIn,
        IDLE
    }

    enum AutoTrajectoryCenter {
        Start,
        AutoTrajectoryCenter_Purple_Yellow,
        CenterPathPicking_Center,
        CenterPathPlacing_Center,
        WhiteDrop_Center,
        CenterPathPicking2_Center,
        CenterPathPlacing2_Center,
        WhiteDrop2_Center,
        ParkingIn,
        IDLE
    }

    enum AutoTrajectoryRight {
        Start,
        AutoTrajectoryRight_Purple_Yellow,
        CenterPathPicking_Right,
        CenterPathPlacing_Right,
        WhiteDrop_Right,
        CenterPathPicking2_Right,
        CenterPathPlacing2_Right,
        WhiteDrop2_Right,
        ParkingIn,
        IDLE
    }

    AutoTrajectoryLeft currentLeftState = AutoTrajectoryLeft.Start;
    AutoTrajectoryCenter currentCenterState = AutoTrajectoryCenter.Start;
    AutoTrajectoryRight currentRightState = AutoTrajectoryRight.Start;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        slider = new Slider(hardwareMap, telemetry);
        arm = new ArmV2(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        Pose2d startPose=new Pose2d(14, -62, -Math.PI);
        drive.setPoseEstimate(startPose);
        initTfod();


        //TODO Left Trajectories
        TrajectorySequence AutoTrajectoryLeft_Purple_Yellow = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                .lineToConstantHeading(PurpleLeft)

                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{arm.setArmPos(0.4, 0.175);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.45);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);arm.setArmPos(0.54, 0.66);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(YellowLeft, 0)

                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{ArmV2.DropPixel(1);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.636);Intake.intakeWristServo.setPosition(0.262 + wristPlay1);})
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {arm.setArmPos(0.3, 0.175);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(0.15, 0.175);})
                .setReversed(false)
                .resetConstraints()
                .build();

        TrajectorySequence CenterPathPicking = drive.trajectorySequenceBuilder(AutoTrajectoryLeft_Purple_Yellow.end())
                .splineToConstantHeading(new Vector2d(18,-7), -Math.PI)
                .splineToConstantHeading(new Vector2d(-33,-10), -Math.PI)

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.16);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.38);})

                .lineToSplineHeading(StackLeftPos)

                .waitSeconds(0.1)
                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{intake.setArm(0.65, 0.4);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setReversed(true)
                .resetConstraints()
                .build();


        TrajectorySequence CenterPathPlacing = drive.trajectorySequenceBuilder(CenterPathPicking.end())
                .splineToConstantHeading(new Vector2d(-34,-12),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.66);Intake.IntakePixel(0.77);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);Intake.IntakePixel(0.77);})
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);Intake.IntakePixel(0.77);})

                .splineToConstantHeading(new Vector2d(36,-12),0) //28

                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.165);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.1, 0.155);slider.extendTo(-10, 1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{slider.extendTo(0, 1);})
                .build();


        TrajectorySequence WhiteDrop_Left = drive.trajectorySequenceBuilder(CenterPathPlacing.end())
                .splineToConstantHeading(new Vector2d(50, -36), 0)
                .lineToConstantHeading(new Vector2d(50.5, -36))

                .UNSTABLE_addTemporalMarkerOffset(-1,()->{arm.setArmPos(0.53, 0.175);})
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{arm.setArmPos(0.53, 0.68);})
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{ArmV2.DropPixel(0.84);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.49, 0.68);})
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.38);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.68);})
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.513);Intake.intakeWristServo.setPosition(0.375 + wristPlay2);})
//                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{ArmV2.DropPixel(1);})
//                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
//                .waitSeconds(0.2) //start
//                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.38);})
//                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
//                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(0.513);Intake.intakeWristServo.setPosition(0.375);})
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.175);})
                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.175);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.175);})
                .resetConstraints()
                .build();

        TrajectorySequence CenterPathPicking2 = drive.trajectorySequenceBuilder(WhiteDrop_Left.end())
                .splineToConstantHeading(new Vector2d(18,-7), -Math.PI)
                .splineToConstantHeading(new Vector2d(-33,-10), -Math.PI)

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.16);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.38);})

                .lineToSplineHeading(StackLeftPos)

                .waitSeconds(0.1)
                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{intake.setArm(0.65, 0.4);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setReversed(true)
                .resetConstraints()
                .build();


        TrajectorySequence CenterPathPlacing2 = drive.trajectorySequenceBuilder(CenterPathPicking2.end())
                .splineToConstantHeading(new Vector2d(-34,-12),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.66);Intake.IntakePixel(0.77);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);Intake.IntakePixel(0.77);})
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);Intake.IntakePixel(0.77);})

                .splineToConstantHeading(new Vector2d(36,-12),0) //28

                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.165);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.1, 0.155);slider.extendTo(-10, 1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{slider.extendTo(0, 1);})
                .build();


        TrajectorySequence WhiteDrop2_Left = drive.trajectorySequenceBuilder(CenterPathPlacing2.end())
                .splineToConstantHeading(new Vector2d(50, -36), 0)
                .lineToConstantHeading(new Vector2d(50.5, -36))

                .UNSTABLE_addTemporalMarkerOffset(-1,()->{arm.setArmPos(0.53, 0.175);})
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{arm.setArmPos(0.53, 0.68);})
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{ArmV2.DropPixel(0.84);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.49, 0.68);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.68);})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.38);})
                .UNSTABLE_addTemporalMarkerOffset(0.4,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.175);})
                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.175);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.175);})
                .resetConstraints()
                .build();

        TrajectorySequence ParkingOut = drive.trajectorySequenceBuilder(WhiteDrop2_Left.end())
                .lineToSplineHeading(new Pose2d(50, -10, Math.PI/2))
                .lineToConstantHeading(new Vector2d(60, -10))
                .build();

        TrajectorySequence ParkingIn = drive.trajectorySequenceBuilder(WhiteDrop_Left.end())
                .lineToConstantHeading(new Vector2d(48, -60))
                .turn(-Math.PI/2)
                .build();




        //TODO Center Trajectories
        TrajectorySequence AutoTrajectoryCenter_Purple_Yellow = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                .lineToConstantHeading(PurpleCenter)

                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{arm.setArmPos(0.4, 0.16);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.5);})
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.54, 0.66);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(YellowCenter, 0)

                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{ArmV2.DropPixel(1);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.636);Intake.intakeWristServo.setPosition(0.262 + wristPlay1);})
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {arm.setArmPos(0.3, 0.175);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(0.15, 0.175);})
                .setReversed(false)
                .resetConstraints()
                .build();

        TrajectorySequence CenterPathPicking_Center = drive.trajectorySequenceBuilder(AutoTrajectoryCenter_Purple_Yellow.end())
                .build();
        TrajectorySequence CenterPathPlacing_Center = drive.trajectorySequenceBuilder(CenterPathPicking_Center.end())
                .build();
        TrajectorySequence WhiteDrop_Center = drive.trajectorySequenceBuilder(CenterPathPlacing_Center.end())
                .build();
        TrajectorySequence CenterPathPicking2_Center = drive.trajectorySequenceBuilder(WhiteDrop_Center.end())
                .build();
        TrajectorySequence CenterPathPlacing2_Center = drive.trajectorySequenceBuilder(CenterPathPicking2_Center.end())
                .build();
        TrajectorySequence WhiteDrop2_Center = drive.trajectorySequenceBuilder(CenterPathPlacing2_Center.end())
                .build();




        //TODO Right Trajectories
        TrajectorySequence AutoTrajectoryRight_Purple_Yellow = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                .lineToConstantHeading(PurpleRight)

                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .addTemporalMarker(()->{arm.setArmPos(0.4, 0.16);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.66);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(YellowRight, 0)

                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{ArmV2.DropPixel(1);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.636);Intake.intakeWristServo.setPosition(0.262 + wristPlay1);})
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {arm.setArmPos(0.3, 0.175);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(0.15, 0.175);})
                .setReversed(false)
                .resetConstraints()
                .build();

        TrajectorySequence CenterPathPicking_Right = drive.trajectorySequenceBuilder(AutoTrajectoryRight_Purple_Yellow.end())
                .build();
        TrajectorySequence CenterPathPlacing_Right = drive.trajectorySequenceBuilder(CenterPathPicking_Right.end())
                .build();
        TrajectorySequence WhiteDrop_Right = drive.trajectorySequenceBuilder(CenterPathPlacing_Right.end())
                .build();
        TrajectorySequence CenterPathPicking2_Right = drive.trajectorySequenceBuilder(WhiteDrop_Right.end())
                .build();
        TrajectorySequence CenterPathPlacing2_Right = drive.trajectorySequenceBuilder(CenterPathPicking2_Right.end())
                .build();
        TrajectorySequence WhiteDrop2_Right = drive.trajectorySequenceBuilder(CenterPathPlacing2_Right.end())
                .build();


        while (opModeInInit()) {
            Intake.IntakePixel(0.8);
            sleep(100);
            slider.extendToHome();
            ArmV2.SetArmPosition(0.15, 0.16);
            Intake.SetArmPosition(0.5, 0.66);
            ArmV2.DropPixel(0.8);
            Intake.CrankPosition(0.69);
            ArmV2.SliderLink(0.95);

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());
            if (currentRecognitions.size() != 0) {

                boolean objectFound = false;

                for (Recognition recognition : currentRecognitions) {
                    x = (recognition.getLeft() + recognition.getRight()) / 2;
                    y = (recognition.getTop() + recognition.getBottom()) / 2;

                    objectFound = true;

                    telemetry.addLine("Beacon");
                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position", "%.0f / %.0f", x, y);
                    telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                    telemetry.update();

                    break;
                }

                if (objectFound) {

//                    Adjust values according to your bot and camera position
                    if (x >= 800 && x <= 1100) {
                        propPosition = "right";
                    } else if (x >= 500 && x <= 790) {
                        propPosition = "center";
                    } else if (x >= 200 && x <= 490) {
                        propPosition = "left";
                    }


                } else {
                    telemetry.addLine("Don't see the beacon :(");
                }
                telemetry.addData("position", propPosition);
                telemetry.update();
                telemetry.update();
            }
        }

        waitForStart();

        while (opModeIsActive()) {

            //LEFT TRAJECTORY
            switch (currentLeftState){
                case Start:
                    if (gamepad1.x || propPosition == "left"){
                        if(!drive.isBusy()){
                            currentLeftState = AutoTrajectoryLeft.AutoTrajectoryLeft_Purple_Yellow;
                            drive.followTrajectorySequenceAsync(AutoTrajectoryLeft_Purple_Yellow);
                        }
                    }
                    break;
                case AutoTrajectoryLeft_Purple_Yellow:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.CenterPathPicking;
                        drive.followTrajectorySequenceAsync(CenterPathPicking);
                    }
                    break;
                case CenterPathPicking:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.CenterPathPlacing;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing);
                    }
                    break;
                case CenterPathPlacing:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.WhiteDrop_Left;
                        drive.followTrajectorySequenceAsync(WhiteDrop_Left);
                    }
                    break;
                case WhiteDrop_Left:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.CenterPathPicking2;
                        drive.followTrajectorySequenceAsync(CenterPathPicking2);
                    }
                    break;
                case CenterPathPicking2:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.CenterPathPlacing2;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing2);
                    }
                    break;
                case CenterPathPlacing2:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.WhiteDrop2_Left;
                        drive.followTrajectorySequenceAsync(WhiteDrop2_Left);
                    }
                    break;
                case WhiteDrop2_Left:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.ParkingIn;
//                        drive.followTrajectorySequenceAsync(ParkingIn);
                    }
                    break;
                case ParkingIn:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }




            //CENTER TRAJECTORY
//            switch (currentCenterState){
//                case Start:
//                    if (gamepad1.y || propPosition == "center"){
//                        if(!drive.isBusy()){
//                            currentCenterState = AutoTrajectoryCenter.AutoTrajectoryCenter_Purple_Yellow;
//                            drive.followTrajectorySequenceAsync(AutoTrajectoryCenter_Purple_Yellow);
//                        }
//                    }
//                    break;
//                case AutoTrajectoryCenter_Purple_Yellow:
//                    if(!drive.isBusy()){
//                        currentCenterState = AutoTrajectoryCenter.CenterPathPicking_Center;
//                        drive.followTrajectorySequenceAsync(CenterPathPicking_Center);
//                    }
//                    break;
//                case CenterPathPicking_Center:
//                    if(!drive.isBusy()){
//                        currentCenterState = AutoTrajectoryCenter.CenterPathPlacing_Center;
//                        drive.followTrajectorySequenceAsync(CenterPathPlacing_Center);
//                    }
//                    break;
//                case CenterPathPlacing_Center:
//                    if(!drive.isBusy()){
//                        currentCenterState = AutoTrajectoryCenter.WhiteDrop_Center;
//                        drive.followTrajectorySequenceAsync(WhiteDrop_Center);
//                    }
//                    break;
//                case WhiteDrop_Center:
//                    if(!drive.isBusy()){
//                        currentCenterState = AutoTrajectoryCenter.CenterPathPicking2_Center;
//                        drive.followTrajectorySequenceAsync(CenterPathPicking2_Center);
//                    }
//                    break;
//                case CenterPathPicking2_Center:
//                    if(!drive.isBusy()){
//                        currentCenterState = AutoTrajectoryCenter.CenterPathPlacing2_Center;
//                        drive.followTrajectorySequenceAsync(CenterPathPlacing2_Center);
//                    }
//                    break;
//                case CenterPathPlacing2_Center:
//                    if(!drive.isBusy()){
//                        currentCenterState = AutoTrajectoryCenter.WhiteDrop2_Center;
//                        drive.followTrajectorySequenceAsync(WhiteDrop2_Center);
//                    }
//                    break;
//                case WhiteDrop2_Center:
//                    if(!drive.isBusy()){
//                        currentCenterState = AutoTrajectoryCenter.ParkingIn;
////                        drive.followTrajectorySequenceAsync(ParkingIn);
//                    }
//                    break;
//                case ParkingIn:
//                    if(!drive.isBusy()){
//                        currentCenterState = AutoTrajectoryCenter.IDLE;
//                    }
//                    break;
//                case IDLE:
//                    break;
//            }




            //RIGHT TRAJECTORY
//            switch (currentRightState){
//                case Start:
//                    if (gamepad1.b || propPosition == "right"){
//                        if(!drive.isBusy()){
//                            currentRightState = AutoTrajectoryRight.AutoTrajectoryRight_Purple_Yellow;
//                            drive.followTrajectorySequenceAsync(AutoTrajectoryRight_Purple_Yellow);
//                        }
//                    }
//                    break;
//                case AutoTrajectoryRight_Purple_Yellow:
//                    if(!drive.isBusy()){
//                        currentRightState = AutoTrajectoryRight.CenterPathPicking_Right;
//                        drive.followTrajectorySequenceAsync(CenterPathPicking_Right);
//                    }
//                    break;
//                case CenterPathPicking_Right:
//                    if(!drive.isBusy()){
//                        currentRightState = AutoTrajectoryRight.CenterPathPlacing_Right;
//                        drive.followTrajectorySequenceAsync(CenterPathPlacing_Right);
//                    }
//                    break;
//                case CenterPathPlacing_Right:
//                    if(!drive.isBusy()){
//                        currentRightState = AutoTrajectoryRight.WhiteDrop_Right;
//                        drive.followTrajectorySequenceAsync(WhiteDrop_Right);
//                    }
//                    break;
//                case WhiteDrop_Right:
//                    if(!drive.isBusy()){
//                        currentRightState = AutoTrajectoryRight.CenterPathPicking2_Right;
//                        drive.followTrajectorySequenceAsync(CenterPathPicking2_Right);
//                    }
//                    break;
//                case CenterPathPicking2_Right:
//                    if(!drive.isBusy()){
//                        currentRightState = AutoTrajectoryRight.CenterPathPlacing2_Right;
//                        drive.followTrajectorySequenceAsync(CenterPathPlacing2_Right);
//                    }
//                    break;
//                case CenterPathPlacing2_Right:
//                    if(!drive.isBusy()){
//                        currentRightState = AutoTrajectoryRight.WhiteDrop2_Right;
//                        drive.followTrajectorySequenceAsync(WhiteDrop2_Right);
//                    }
//                    break;
//                case WhiteDrop2_Right:
//                    if(!drive.isBusy()){
//                        currentRightState = AutoTrajectoryRight.ParkingIn;
////                        drive.followTrajectorySequenceAsync(ParkingIn);
//                    }
//                    break;
//                case ParkingIn:
//                    if(!drive.isBusy()){
//                        currentRightState = AutoTrajectoryRight.IDLE;
//                    }
//                    break;
//                case IDLE:
//                    break;
//            }


            telemetry.addData("LeftFrontCurrent", drive.getMotorCurrent().get(0));
            telemetry.addData("RightFrontCurrent", drive.getMotorCurrent().get(1));
            telemetry.addData("LeftRearCurrent", drive.getMotorCurrent().get(2));
            telemetry.addData("RightRearCurrent", drive.getMotorCurrent().get(3));
            telemetry.addData("X-Pos", drive.getPoseEstimate().getX());
            telemetry.addData("YPos", drive.getPoseEstimate().getY());
            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
            telemetry.addData("position",propPosition);
            drive.update();
            telemetry.update();
        }
        visionPortal.close();
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.


        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        telemetry.addLine("init done");
        telemetry.update();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.92f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

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
