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
import org.firstinspires.ftc.teamcode.vision.Globals;
import org.firstinspires.ftc.teamcode.vision.Location;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "REDNearAuto_1Cycle", group = "2Cycle_Autos")
//@Disabled
public class RedNearAuto extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    ArmV2 arm = null;
    Hanger hanger = null;
    Intake intake = null;
    public static int val=0;
    double x;
    double y;
    String propPosition = " ";

    public static double
            lifter_posL = 0, lifter_posR = 0, error_lifter, error_diff, error_int, error_lifterR, error_diffR, error_intR, errorprev, errorprevR, output_lifter, output_lifterR, output_power, target, dropVal;

    public static double kp = 4, ki, kd = 1.7;

    public static double stackDiff = 0.5;

    public static Pose2d PurpleLeftPos, YellowLeftPos, StackLeftPos = new Pose2d(-53 - stackDiff, -10, -Math.PI); //-51,-12
    public static Vector2d PurpleLeft = new Vector2d(16 , -29), YellowLeft = new Vector2d(50,-26), StackLeft; //25

    public static Pose2d PurpleCenterPos, YellowCenterPos, StackCenterPos;
    public static Vector2d PurpleCenter = new Vector2d(25 , -21), YellowCenter = new Vector2d(53,-35), StackCenter;

    public static Pose2d PurpleRightPos, YellowRightPos, StackRightPos;
    public static Vector2d PurpleRight = new Vector2d(35 , -30), YellowRight = new Vector2d(52,-41), StackRight;

    public static double wristPlay1 = -0.01, wristPlay2 = 0.00;

    public static double armServoOnePos = 0.92, armServoOneUP = 0.8, armServoOneOut = 0.47;
    public static double yellowDiff = 3.5;
    private PropPipeline propPipeline;
    private VisionPortal portal;
    private Location randomization;

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

        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Location.RED;
        Globals.SIDE = Location.CLOSE;

        Pose2d startPose=new Pose2d(14, -62, -Math.PI);
        drive.setPoseEstimate(startPose);


        //TODO Left Trajectories
        TrajectorySequence AutoTrajectoryLeft_Purple_Yellow = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                //backdrop
                .lineToConstantHeading(new Vector2d(16 , -29))
                .addTemporalMarker(()->{Intake.CrankPosition(0.35);arm.setArmPos(armServoOneUP, 0.16);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Intake.CrankPosition(0.48);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .lineToConstantHeading(new Vector2d(50.5 - yellowDiff,-28))
                .waitSeconds(0.3)
                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.68);})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.4)
                .resetConstraints()
                .addTemporalMarker(() -> {arm.setArmPos(0.5, 0.68);})
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {arm.setArmPos(armServoOneUP, 0.16);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(armServoOnePos, 0.16);})
                .waitSeconds(0.1)
                .build();

        TrajectorySequence CenterPathPicking = drive.trajectorySequenceBuilder(AutoTrajectoryLeft_Purple_Yellow.end())
                .splineToConstantHeading(new Vector2d(18,-7), -Math.PI)
                .splineToConstantHeading(new Vector2d(-34,-10), -Math.PI)
                .waitSeconds(0.1)

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.16);})
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.650);Intake.intakeWristServo.setPosition(0.25 + wristPlay1);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.45);})

                .lineToSplineHeading(StackLeftPos)

                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.650);Intake.intakeWristServo.setPosition(0.23 + wristPlay1);})
                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{intake.setArm(0.65, 0.4);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setReversed(true)
                .resetConstraints()
                .build();


        TrajectorySequence CenterPathPlacing = drive.trajectorySequenceBuilder(CenterPathPicking.end())
                .splineToConstantHeading(new Vector2d(-34,-10),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.66);Intake.IntakePixel(0.77);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);Intake.IntakePixel(0.77);})
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);Intake.IntakePixel(0.77);})

                .splineToConstantHeading(new Vector2d(36,-10),0) //28

                .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, 0.165);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.155);slider.extendTo(-10, 1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{slider.extendTo(0, 1);})
                .build();


        TrajectorySequence WhiteDrop_Left = drive.trajectorySequenceBuilder(CenterPathPlacing.end())
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.175);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.6);ArmV2.SliderLink(0.5);})
                .splineToConstantHeading(new Vector2d(45, -36), 0)
                .lineToConstantHeading(new Vector2d(45.5, -36))

//                .UNSTABLE_addTemporalMarkerOffset(-1,()->{arm.setArmPos(armServoOneOut, 0.175);})
//                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{arm.setArmPos(armServoOneOut, 0.68);ArmV2.SliderLink(0.5);})
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{ArmV2.DropPixel(0.84);})
                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.68);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ArmV2.DropPixel(0.84);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.68);})
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.38);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.68);ArmV2.SliderLink(0.2);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);ArmV2.SliderLink(0.95);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.513);Intake.intakeWristServo.setPosition(0.375 + wristPlay2);})
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.175);})
                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.175);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, 0.175);})
                .resetConstraints()
                .build();

//        TrajectorySequence CenterPathPicking2 = drive.trajectorySequenceBuilder(WhiteDrop_Left.end())
//                .splineToConstantHeading(new Vector2d(18,-7), -Math.PI)
//                .splineToConstantHeading(new Vector2d(-33,-10), -Math.PI)
//
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.16);})
//                .addTemporalMarker(()->{Intake.CrankPosition(0.38);})
//
//                .lineToSplineHeading(StackLeftPos)
//
//                .waitSeconds(0.1)
//                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{intake.setArm(0.65, 0.4);})
//                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
//                .setReversed(true)
//                .resetConstraints()
//                .build();
//
//
//        TrajectorySequence CenterPathPlacing2 = drive.trajectorySequenceBuilder(CenterPathPicking2.end())
//                .splineToConstantHeading(new Vector2d(-34,-12),0)
//
//                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.66);Intake.IntakePixel(0.77);})
//                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);Intake.IntakePixel(0.77);})
//                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);Intake.IntakePixel(0.77);})
//
//                .splineToConstantHeading(new Vector2d(36,-12),0) //28
//
//                .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, 0.165);})
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.155);slider.extendTo(-10, 1);})
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{slider.extendTo(0, 1);})
//                .build();
//
//
//        TrajectorySequence WhiteDrop2_Left = drive.trajectorySequenceBuilder(CenterPathPlacing2.end())
//                .splineToConstantHeading(new Vector2d(50, -36), 0)
//                .lineToConstantHeading(new Vector2d(50.5, -36))
//
//                .UNSTABLE_addTemporalMarkerOffset(-1,()->{arm.setArmPos(0.53, 0.175);})
//                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{arm.setArmPos(0.53, 0.68);})
//                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{ArmV2.DropPixel(0.84);})
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.68);})
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
//                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.68);})
//                .waitSeconds(0.3)
//                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.38);})
//                .UNSTABLE_addTemporalMarkerOffset(0.4,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
//                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.175);})
//                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.175);})
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, 0.175);})
//                .resetConstraints()
//                .build();

        TrajectorySequence ParkingOut = drive.trajectorySequenceBuilder(WhiteDrop_Left.end())
                .lineToSplineHeading(new Pose2d(50, -10, Math.PI/2))
                .lineToConstantHeading(new Vector2d(60, -10))
                .build();

        TrajectorySequence ParkingIn = drive.trajectorySequenceBuilder(WhiteDrop_Left.end())
                .lineToConstantHeading(new Vector2d(48, -60))
//                .turn(-Math.PI/2)
                .build();




        //TODO Center Trajectories
        TrajectorySequence AutoTrajectoryCenter_Purple_Yellow = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                //backdrop
                .lineToConstantHeading(new Vector2d(25 , -23))
                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.16);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.5);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
//                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.IntakePixel(1);})
                .waitSeconds(0.7)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.57);})
//                .waitSeconds(0.5)
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .splineToConstantHeading(new Vector2d(50.5 - yellowDiff,-33.5), 0)
                .lineToConstantHeading(new Vector2d(50.5 - yellowDiff,-35)) //33.5
                .waitSeconds(0.3)
                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.68);})
                .waitSeconds(1)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(1)
                .resetConstraints()
                .addTemporalMarker(() -> {arm.setArmPos(0.5, 0.68);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(armServoOneUP, 0.16);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(armServoOnePos, 0.16);})
                .waitSeconds(0.1)
                .setReversed(false)
                .build();

        TrajectorySequence CenterPathPicking_Center = drive.trajectorySequenceBuilder(AutoTrajectoryCenter_Purple_Yellow.end())
                .splineToConstantHeading(new Vector2d(18,-7), -Math.PI)
                .splineToConstantHeading(new Vector2d(-34,-10), -Math.PI)
                .waitSeconds(0.1)

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.16);})
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.650);Intake.intakeWristServo.setPosition(0.25 + wristPlay1);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.45);})

                .lineToSplineHeading(StackLeftPos)

                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.650);Intake.intakeWristServo.setPosition(0.23 + wristPlay1);})
                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{intake.setArm(0.65, 0.4);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setReversed(true)
                .resetConstraints()
                .build();

        TrajectorySequence CenterPathPlacing_Center = drive.trajectorySequenceBuilder(CenterPathPicking_Center.end())
                .splineToConstantHeading(new Vector2d(-34,-10),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.66);Intake.IntakePixel(0.77);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);Intake.IntakePixel(0.77);})
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);Intake.IntakePixel(0.77);})

                .splineToConstantHeading(new Vector2d(36,-10),0) //28

                .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, 0.165);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.155);slider.extendTo(-10, 1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{slider.extendTo(0, 1);})
                .build();

        TrajectorySequence WhiteDrop_Center = drive.trajectorySequenceBuilder(CenterPathPlacing_Center.end())
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.175);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.6);ArmV2.SliderLink(0.5);})

                .splineToConstantHeading(new Vector2d(45, -31), 0)
                .lineToConstantHeading(new Vector2d(45.5, -31))

                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.68);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ArmV2.DropPixel(0.84);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.68);})
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.38);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.68);ArmV2.SliderLink(0.2);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);ArmV2.SliderLink(0.95);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.513);Intake.intakeWristServo.setPosition(0.375 + wristPlay2);})
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.175);})
                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.175);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, 0.175);})
                .resetConstraints()
                .build();


//        TrajectorySequence CenterPathPicking2_Center = drive.trajectorySequenceBuilder(WhiteDrop_Center.end())
//                .build();
//        TrajectorySequence CenterPathPlacing2_Center = drive.trajectorySequenceBuilder(CenterPathPicking2_Center.end())
//                .build();
//        TrajectorySequence WhiteDrop2_Center = drive.trajectorySequenceBuilder(CenterPathPlacing2_Center.end())
//                .build();




        //TODO Right Trajectories
        TrajectorySequence AutoTrajectoryRight_Purple_Yellow = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                //backdrop
                .lineToConstantHeading(new Vector2d(32 , -30))
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.16);})
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .splineToConstantHeading(new Vector2d(51 - yellowDiff,-41), 0)
                .lineToConstantHeading(new Vector2d(51 - yellowDiff,-41))
                .waitSeconds(0.3)
                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.64);})
                .waitSeconds(1)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.67);})
                .waitSeconds(1)//0.55
                .resetConstraints()

                //pixel intake // round 1
                .addTemporalMarker(() -> {arm.setArmPos(0.5, 0.68);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(armServoOneUP, 0.16);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(armServoOnePos, 0.16);})
                .waitSeconds(0.2)
                .build();

        TrajectorySequence CenterPathPicking_Right = drive.trajectorySequenceBuilder(AutoTrajectoryRight_Purple_Yellow.end())
                .splineToConstantHeading(new Vector2d(34,-7), -Math.PI) //18
                .splineToConstantHeading(new Vector2d(-34,-10), -Math.PI)
                .waitSeconds(0.1)

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.16);})
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.650);Intake.intakeWristServo.setPosition(0.25 + wristPlay1);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.45);})

                .lineToSplineHeading(StackLeftPos)

                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.650);Intake.intakeWristServo.setPosition(0.23 + wristPlay1);})
                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{intake.setArm(0.65, 0.4);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setReversed(true)
                .resetConstraints()
                .build();

        TrajectorySequence CenterPathPlacing_Right = drive.trajectorySequenceBuilder(CenterPathPicking_Right.end())
                .splineToConstantHeading(new Vector2d(-34,-10),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.66);Intake.IntakePixel(0.77);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);Intake.IntakePixel(0.77);})
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);Intake.IntakePixel(0.77);})

                .splineToConstantHeading(new Vector2d(44,-10),0) //28

                .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, 0.165);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.155);slider.extendTo(-10, 1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{slider.extendTo(0, 1);})
                .build();

        TrajectorySequence WhiteDrop_Right = drive.trajectorySequenceBuilder(CenterPathPlacing_Right.end())
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.175);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.6);ArmV2.SliderLink(0.5);})

                .splineToConstantHeading(new Vector2d(45, -31), 0)
                .lineToConstantHeading(new Vector2d(45.5, -31))

                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.68);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ArmV2.DropPixel(0.84);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.68);})
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.38);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.68);ArmV2.SliderLink(0.2);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);ArmV2.SliderLink(0.95);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.513);Intake.intakeWristServo.setPosition(0.375 + wristPlay2);})
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.175);})
                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.175);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, 0.175);})
                .resetConstraints()
                .build();

//        TrajectorySequence CenterPathPicking2_Right = drive.trajectorySequenceBuilder(WhiteDrop_Right.end())
//                .build();
//        TrajectorySequence CenterPathPlacing2_Right = drive.trajectorySequenceBuilder(CenterPathPicking2_Right.end())
//                .build();
//        TrajectorySequence WhiteDrop2_Right = drive.trajectorySequenceBuilder(CenterPathPlacing2_Right.end())
//                .build();



        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
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

        while (opModeInInit()) {
            Intake.IntakePixel(0.8);
            sleep(100);
            slider.extendToHome();
            ArmV2.SetArmPosition(armServoOnePos, 0.16);
            Intake.SetArmPosition(0.5, 0.66);
            ArmV2.DropPixel(0.8);
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

            //LEFT TRAJECTORY
            switch (currentLeftState){
                case Start:
                    if (gamepad1.x || propPosition == "LEFT"){
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
//                        currentLeftState = AutoTrajectoryLeft.CenterPathPicking2;
//                        drive.followTrajectorySequenceAsync(CenterPathPicking2);

                        currentLeftState = AutoTrajectoryLeft.ParkingIn;
                        drive.followTrajectorySequenceAsync(ParkingIn);
                    }
                    break;
//                case CenterPathPicking2:
//                    if(!drive.isBusy()){
//                        currentLeftState = AutoTrajectoryLeft.CenterPathPlacing2;
//                        drive.followTrajectorySequenceAsync(CenterPathPlacing2);
//                    }
//                    break;
//                case CenterPathPlacing2:
//                    if(!drive.isBusy()){
//                        currentLeftState = AutoTrajectoryLeft.WhiteDrop2_Left;
//                        drive.followTrajectorySequenceAsync(WhiteDrop2_Left);
//                    }
//                    break;
//                case WhiteDrop2_Left:
//                    if(!drive.isBusy()){
//                        currentLeftState = AutoTrajectoryLeft.ParkingIn;
//                        drive.followTrajectorySequenceAsync(ParkingIn);
//                    }
//                    break;
                case ParkingIn:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }




            //CENTER TRAJECTORY
            switch (currentCenterState){
                case Start:
                    if (gamepad1.y || propPosition == "CENTER"){
                        if(!drive.isBusy()){
                            currentCenterState = AutoTrajectoryCenter.AutoTrajectoryCenter_Purple_Yellow;
                            drive.followTrajectorySequenceAsync(AutoTrajectoryCenter_Purple_Yellow);
                        }
                    }
                    break;
                case AutoTrajectoryCenter_Purple_Yellow:
                    if(!drive.isBusy()){
                        currentCenterState = AutoTrajectoryCenter.CenterPathPicking_Center;
                        drive.followTrajectorySequenceAsync(CenterPathPicking_Center);
                    }
                    break;
                case CenterPathPicking_Center:
                    if(!drive.isBusy()){
                        currentCenterState = AutoTrajectoryCenter.CenterPathPlacing_Center;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing_Center);
                    }
                    break;
                case CenterPathPlacing_Center:
                    if(!drive.isBusy()){
                        currentCenterState = AutoTrajectoryCenter.WhiteDrop_Center;
                        drive.followTrajectorySequenceAsync(WhiteDrop_Center);
                    }
                    break;
                case WhiteDrop_Center:
                    if(!drive.isBusy()){
                        currentCenterState = AutoTrajectoryCenter.ParkingIn;
                        drive.followTrajectorySequenceAsync(ParkingIn);
                    }
                    break;
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
                case ParkingIn:
                    if(!drive.isBusy()){
                        currentCenterState = AutoTrajectoryCenter.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }




            //RIGHT TRAJECTORY
            switch (currentRightState){
                case Start:
                    if (gamepad1.b || propPosition == "RIGHT"){
                        if(!drive.isBusy()){
                            currentRightState = AutoTrajectoryRight.AutoTrajectoryRight_Purple_Yellow;
                            drive.followTrajectorySequenceAsync(AutoTrajectoryRight_Purple_Yellow);
                        }
                    }
                    break;
                case AutoTrajectoryRight_Purple_Yellow:
                    if(!drive.isBusy()){
                        currentRightState = AutoTrajectoryRight.CenterPathPicking_Right;
                        drive.followTrajectorySequenceAsync(CenterPathPicking_Right);
                    }
                    break;
                case CenterPathPicking_Right:
                    if(!drive.isBusy()){
                        currentRightState = AutoTrajectoryRight.CenterPathPlacing_Right;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing_Right);
                    }
                    break;
                case CenterPathPlacing_Right:
                    if(!drive.isBusy()){
                        currentRightState = AutoTrajectoryRight.WhiteDrop_Right;
                        drive.followTrajectorySequenceAsync(WhiteDrop_Right);
                    }
                    break;
                case WhiteDrop_Right:
                    if(!drive.isBusy()){
                        currentRightState = AutoTrajectoryRight.ParkingIn;
                        drive.followTrajectorySequenceAsync(ParkingIn);
                    }
                    break;
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
                case ParkingIn:
                    if(!drive.isBusy()){
                        currentRightState = AutoTrajectoryRight.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }


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
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    public VisionPortal.CameraState getCameraState() {
        if (portal != null) return portal.getCameraState();
        return null;
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
