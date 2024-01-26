package org.firstinspires.ftc.teamcode.autos.cycle_autos;

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

@Autonomous(name="REDFarAuto_1Cycle", group = "2Cycle_Autos")//(name="RedFarAuto_1Cycle", group = "Cycle_Autos")
@Config
//@Disabled
public class RedFarAuto extends LinearOpMode {
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
    public static double yellowDiff = 5.0;
    public static Pose2d PurpleRightPos = new Pose2d(-38,-32, -Math.PI), YellowRightPos, StackRightPos = new Pose2d(-53 , -12 + stackDiff, -Math.PI); //53
    public static Vector2d PurpleRight, YellowRight = new Vector2d(50 - yellowDiff,-41.5), StackRight = new Vector2d(-51, -12.5);

    public static Pose2d PurpleLeftPos = new Pose2d(-40,-30, 0), YellowLeftPos, StackLeftPos = new Pose2d(-53 , -12.5 + stackDiff, -Math.PI);//54, //-44
    public static Vector2d PurpleLeft, YellowLeft = new Vector2d(50 - yellowDiff,-28.5), StackLeft = new Vector2d(-51, -12); //48

    public static Pose2d PurpleCenterPos = new Pose2d(-53,-24, 0), YellowCenterPos, StackCenterPos = new Pose2d(-52 , -12, -Math.PI); //51
    public static Vector2d PurpleCenter, YellowCenter = new Vector2d(50 - yellowDiff,-36), StackCenter = new Vector2d(-51, -12); //38


    public static double wristPlay1 = -0.01, wristPlay2 = -0.01;
    public static double armServoOnePos = 0.92, armServoOneUP = 0.8, armServoOneOut = 0.48;


    public enum AutoTrajectoryRight {
        Start,
        AutoTrajectoryRightPurple,
        CenterPathPlacing,
        AutoTrajectoryRightYellow,
        CenterPathPicking,
        CenterPathPlacing2,
        AutoTrajectoryRightYellow2,
        ParkingOut,
        IDLE
    }
    enum AutoTrajectoryCenter {
        Start,
        AutoTrajectoryCenterPurple,
        CenterPathPlacing_Center,
        AutoTrajectoryCenterYellow,
        CenterPathPicking_Center,
        CenterPathPlacing_Center2,
        AutoTrajectoryCenterYellow2,
        ParkingOut,
        IDLE
    }
    enum AutoTrajectoryLeft {
        Start,
        AutoTrajectoryLeftPurple,
        CenterPathPlacing_Left,
        AutoTrajectoryLeftYellow,
        CenterPathPicking_Left,
        CenterPathPlacing_Left2,
        AutoTrajectoryLeftYellow2,
        ParkingOut,
        IDLE
    }
    AutoTrajectoryRight currentState = AutoTrajectoryRight.Start;
    AutoTrajectoryCenter currentState2 = AutoTrajectoryCenter.Start;
    AutoTrajectoryLeft currentState3 = AutoTrajectoryLeft.Start;

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
        Globals.ALLIANCE = Location.RED;
        Globals.SIDE = Location.FAR;

        Pose2d startPose=new Pose2d(-39, -64, 0);
        drive.setPoseEstimate(startPose);


        // TODO Left Trajectories
        TrajectorySequence AutoTrajectoryLeftPurple = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.55);})

                .lineToSplineHeading(PurpleRightPos)

                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{intake.setArm(0.5, 0.66);})
                .build();

        TrajectorySequence CenterPathPlacing_Left = drive.trajectorySequenceBuilder(AutoTrajectoryLeftPurple.end())
                .lineToSplineHeading(new Pose2d(-34 , -12, -Math.PI))

                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.67);Intake.intakeWristServo.setPosition(0.24 + wristPlay1);})
                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.16);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.41);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(30))
                .lineToSplineHeading(StackRightPos)

                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{intake.setArm(0.69, 0.4);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setReversed(true)
                .resetConstraints()

                .splineToConstantHeading(new Vector2d(-34,-12),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);})
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{arm.setArmPos(armServoOnePos, 0.165);})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.155);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(1.4, ()->{slider.extendTo(0, 1);})

                .splineToConstantHeading(new Vector2d(36,-12),0) //28
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{arm.setArmPos(armServoOnePos, 0.165);})
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.155);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{slider.extendTo(0, 1);})
                .build();

        TrajectorySequence AutoTrajectoryLeftYellow = drive.trajectorySequenceBuilder(CenterPathPlacing_Left.end())
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.175);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.6);ArmV2.SliderLink(0.5);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(30))
                .lineToConstantHeading(new Vector2d(YellowLeft.getX(), -36))

//                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{arm.setArmPos(armServoOnePos, 0.165);})
//                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.15);slider.extendTo(-10, 1);})
//                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{slider.extendTo(0, 1);})

//                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.165);})
//                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.65);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{ArmV2.DropPixel(0.84);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.52, 0.65);}) //0.51
//                .forward(1.5)
                .waitSeconds(0.1)

                .lineToConstantHeading(YellowLeft)

                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut,0.635);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.38);})
                .UNSTABLE_addTemporalMarkerOffset(0.4,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.635);})
                .forward(1)
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.7, 0.175);ArmV2.SliderLink(0.95);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, 0.175);})
                .resetConstraints()
                .build();

        TrajectorySequence CenterPathPicking_Left = drive.trajectorySequenceBuilder(AutoTrajectoryLeftYellow.end())
                //round1
                .splineToConstantHeading(new Vector2d(28, -7), -Math.PI)
                .splineToConstantHeading(new Vector2d(-33, -7), -Math.PI)
                .build();

        TrajectorySequence CenterPathPlacing_Left2 = drive.trajectorySequenceBuilder(CenterPathPicking_Left.end())
                .lineToSplineHeading(new Pose2d(-34 , -12, -Math.PI))

                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.55);Intake.intakeWristServo.setPosition(0.38 + wristPlay2);})
                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.16);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.41);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
                .lineToSplineHeading(StackLeftPos)

                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.55);Intake.intakeWristServo.setPosition(0.35 + wristPlay2);})
                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{intake.setArm(0.69, 0.4);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setReversed(true)
                .resetConstraints()

                .splineToConstantHeading(new Vector2d(-34,-12),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);})
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{arm.setArmPos(armServoOnePos, 0.165);})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.155);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(1.4, ()->{slider.extendTo(0, 1);})

                .splineToConstantHeading(new Vector2d(36,-12),0) //28
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{arm.setArmPos(armServoOnePos, 0.165);})
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.155);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{slider.extendTo(0, 1);})
                .build();

        TrajectorySequence AutoTrajectoryLeftYellow2 = drive.trajectorySequenceBuilder(CenterPathPlacing_Left2.end())
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.175);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.6);ArmV2.SliderLink(0.5);})

                .splineToConstantHeading(new Vector2d(YellowLeft.getX(), -32), 0)
                .lineToConstantHeading(new Vector2d(YellowLeft.getX(), -32).plus(new Vector2d(0.5, 0)))

//                .UNSTABLE_addTemporalMarkerOffset(-1,()->{arm.setArmPos(0.53, 0.175);})
//                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{arm.setArmPos(0.53, 0.68);})
                .addTemporalMarker(()->{ArmV2.DropPixel(0.84);})
                .waitSeconds(0.2)//0.3
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.68);})
                .waitSeconds(0.2)//0.1
                .addTemporalMarker(()->{arm.setArmPos(0.53, 0.68);})
                .waitSeconds(0.2)//0.1
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.2) //start
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.38);})
                .UNSTABLE_addTemporalMarkerOffset(0.4,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .addTemporalMarker(()->{arm.setArmPos(0.50, 0.68);})
                .addTemporalMarker(()->{arm.setArmPos(0.7, 0.175);ArmV2.SliderLink(0.95);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, 0.175);})
                .resetConstraints()
                .build();

        TrajectorySequence ParkingOut = drive.trajectorySequenceBuilder(AutoTrajectoryLeftYellow2.end())
                .lineToSplineHeading(new Pose2d(50, -12, -Math.PI/2))
                .lineToConstantHeading(new Vector2d(60, -12))
                .build();

        TrajectorySequence ParkingIn = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(51, -60))
                .turn(Math.PI/2)
                .build();








        //TODO Center Trajectories
        TrajectorySequence AutoTrajectoryCenterPurple = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.55);})

                .lineToSplineHeading(PurpleCenterPos)

                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.175);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.CrankPosition(0.4);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .addTemporalMarker(()->{intake.setArm(0.5, 0.66);})
                .lineToConstantHeading(new Vector2d(-55, -18))
                .build();

        TrajectorySequence CenterPathPlacing_Center = drive.trajectorySequenceBuilder(AutoTrajectoryCenterPurple.end())
                .lineToSplineHeading(new Pose2d(-34 , -12, -Math.PI))

                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.67);Intake.intakeWristServo.setPosition(0.24 + wristPlay1);})
                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.16);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.41);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(30))
                .lineToSplineHeading(StackRightPos)

                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{intake.setArm(0.69, 0.4);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setReversed(true)
                .resetConstraints()

                .splineToConstantHeading(new Vector2d(-34,-12),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);})
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{arm.setArmPos(armServoOnePos, 0.165);})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.155);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(1.4, ()->{slider.extendTo(0, 1);})

                .splineToConstantHeading(new Vector2d(40,-12),0) //36
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{arm.setArmPos(armServoOnePos, 0.165);})
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.155);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{slider.extendTo(0, 1);})
                .build();

        TrajectorySequence AutoTrajectoryCenterYellow = drive.trajectorySequenceBuilder(CenterPathPlacing_Center.end())
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.175);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.6);ArmV2.SliderLink(0.5);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .lineToConstantHeading(new Vector2d(YellowCenter.getX(), -43))

//                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{arm.setArmPos(armServoOnePos, 0.165);})
//                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.15);slider.extendTo(-10, 1);})
//                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{slider.extendTo(0, 1);})

//                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.165);})
//                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.65);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{ArmV2.DropPixel(0.84);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.65);}) //0.51
                .waitSeconds(0.1)

                .lineToConstantHeading(YellowCenter)

                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut,0.635);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.38);})
                .UNSTABLE_addTemporalMarkerOffset(0.4,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.635);})
                .forward(1)
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.7, 0.175);ArmV2.SliderLink(0.95);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, 0.175);})
                .resetConstraints()
                .build();

        TrajectorySequence CenterPathPicking_Center = drive.trajectorySequenceBuilder(AutoTrajectoryCenterYellow.end())
                //round1
                .splineToConstantHeading(new Vector2d(28, -7), -Math.PI)
                .splineToConstantHeading(new Vector2d(-33, -10), -Math.PI)
                .build();

        TrajectorySequence CenterPathPlacing_Center2 = drive.trajectorySequenceBuilder(CenterPathPicking_Center.end())
                .lineToSplineHeading(new Pose2d(-34 , -12, -Math.PI))

                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.55);Intake.intakeWristServo.setPosition(0.38 + wristPlay2);})
                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.16);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.41);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
                .lineToSplineHeading(StackLeftPos)

                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.55);Intake.intakeWristServo.setPosition(0.35 + wristPlay2);})
                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{intake.setArm(0.69, 0.4);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setReversed(true)
                .resetConstraints()

                .splineToConstantHeading(new Vector2d(-34,-12),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);})
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{arm.setArmPos(armServoOnePos, 0.165);})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.155);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(1.4, ()->{slider.extendTo(0, 1);})

                .splineToConstantHeading(new Vector2d(36,-12),0) //28
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{arm.setArmPos(armServoOnePos, 0.165);})
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.155);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{slider.extendTo(0, 1);})
                .build();

        TrajectorySequence AutoTrajectoryCenterYellow2 = drive.trajectorySequenceBuilder(CenterPathPlacing_Center2.end())
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.175);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.6);ArmV2.SliderLink(0.5);})

                .splineToConstantHeading(new Vector2d(YellowCenter.getX(), -32), 0)
                .lineToConstantHeading(new Vector2d(YellowCenter.getX(), -32).plus(new Vector2d(0.5, 0)))

//                .UNSTABLE_addTemporalMarkerOffset(-1,()->{arm.setArmPos(0.5, 0.175);})
//                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{arm.setArmPos(0.55, 0.68);})
                .addTemporalMarker(()->{ArmV2.DropPixel(0.84);})
                .waitSeconds(0.2) //0.3
                .addTemporalMarker(()->{arm.setArmPos(0.53, 0.68);})
                .waitSeconds(0.3)//0.1
                .forward(1.5)
                .waitSeconds(0.3)//0.1
                .addTemporalMarker(()->{arm.setArmPos(0.52, 0.68);})
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.3) //start
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.38);})
                .UNSTABLE_addTemporalMarkerOffset(0.4,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .addTemporalMarker(()->{arm.setArmPos(0.50, 0.68);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{arm.setArmPos(0.7, 0.175);ArmV2.SliderLink(0.95);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, 0.175);})
                .resetConstraints()
                .build();






        //TODO Right Trajectories
        TrajectorySequence AutoTrajectoryRightPurple = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.55);})

                .lineToSplineHeading(PurpleLeftPos)

                .addTemporalMarker(()->{arm.setArmPos(0.8, 0.175);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.CrankPosition(0.45);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .addTemporalMarker(()->{intake.setArm(0.5, 0.66);})
//                .turn(Math.toRadians(180))
                .build();

        TrajectorySequence CenterPathPlacing = drive.trajectorySequenceBuilder(AutoTrajectoryRightPurple.end())
                .lineToSplineHeading(new Pose2d(-34 , -12, -Math.PI))
                .waitSeconds(0.5)

                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.67);Intake.intakeWristServo.setPosition(0.24 + wristPlay1);})
                .addTemporalMarker(()->{arm.setArmPos(0.8, 0.16);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.45);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(30))
                .lineToSplineHeading(StackRightPos)

                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{intake.setArm(0.69, 0.4);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setReversed(true)
                .resetConstraints()

                .splineToConstantHeading(new Vector2d(-34,-12),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);})
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{arm.setArmPos(armServoOnePos, 0.165);})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.15);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(1.4, ()->{slider.extendTo(0, 1);})

                .splineToConstantHeading(new Vector2d(36,-12),0) //28
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{arm.setArmPos(armServoOnePos, 0.165);})
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.155);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{slider.extendTo(0, 1);})
                .build();

        TrajectorySequence AutoTrajectoryRightYellow = drive.trajectorySequenceBuilder(CenterPathPlacing.end())
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.175);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.6);ArmV2.SliderLink(0.5);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(30))
                .splineToConstantHeading(new Vector2d(YellowRight.getX(), -30), 0)

//                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{arm.setArmPos(armServoOnePos, 0.165);})
//                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.15);slider.extendTo(-10, 1);})
//                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{slider.extendTo(0, 1);})

//                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.165);})
//                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.65);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{ArmV2.DropPixel(0.84);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.54, 0.65);}) //0.51
                .waitSeconds(0.1)

                .lineToConstantHeading(YellowRight)

                .addTemporalMarker(()->{arm.setArmPos(armServoOneOut, 0.65);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.38);})
                .UNSTABLE_addTemporalMarkerOffset(0.4,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .addTemporalMarker(()->{arm.setArmPos(0.49, 0.68);})
                .waitSeconds(0.2)
                .forward(1)
                .addTemporalMarker(()->{arm.setArmPos(0.7, 0.165);ArmV2.SliderLink(0.95);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, 0.175);})
                .resetConstraints()
                .build();

        TrajectorySequence CenterPathPicking = drive.trajectorySequenceBuilder(AutoTrajectoryRightYellow.end())
                //round1
                .splineToConstantHeading(new Vector2d(28, -7), -Math.PI)
                .splineToConstantHeading(new Vector2d(-33, -10), -Math.PI)
                .build();

        TrajectorySequence CenterPathPlacing2 = drive.trajectorySequenceBuilder(CenterPathPicking.end())
                .lineToSplineHeading(new Pose2d(-34 , -12, -Math.PI))

                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.55);Intake.intakeWristServo.setPosition(0.38 + wristPlay2);})
                .addTemporalMarker(()->{arm.setArmPos(armServoOneUP, 0.16);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.41);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
                .lineToSplineHeading(StackLeftPos)

                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.55);Intake.intakeWristServo.setPosition(0.35 + wristPlay2);})
                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{intake.setArm(0.69, 0.4);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setReversed(true)
                .resetConstraints()

                .splineToConstantHeading(new Vector2d(-34,-12),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);})
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{arm.setArmPos(armServoOnePos, 0.165);})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.155);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(1.4, ()->{slider.extendTo(0, 1);})

                .splineToConstantHeading(new Vector2d(42,-12),0) //28
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{arm.setArmPos(armServoOnePos, 0.165);})
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.95, 0.155);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{slider.extendTo(0, 1);})
                .build();

        TrajectorySequence AutoTrajectoryRightYellow2 = drive.trajectorySequenceBuilder(CenterPathPlacing2.end())
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.175);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.6);ArmV2.SliderLink(0.5);})

                .splineToConstantHeading(new Vector2d(YellowRight.getX(), -32), 0)
                .lineToConstantHeading(new Vector2d(YellowRight.getX(), -32).plus(new Vector2d(0.5, 0)))

//                .UNSTABLE_addTemporalMarkerOffset(-1,()->{arm.setArmPos(0.5, 0.175);})
//                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{arm.setArmPos(0.55, 0.68);})
                .addTemporalMarker(()->{ArmV2.DropPixel(0.84);})
                .waitSeconds(0.2) //0.3
                .addTemporalMarker(()->{arm.setArmPos(0.53, 0.68);})
                .waitSeconds(0.3)//0.1
                .forward(1.5)
                .waitSeconds(0.3)//0.1
                .addTemporalMarker(()->{arm.setArmPos(0.52,0.68);})
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.3) //start
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.38);})
                .UNSTABLE_addTemporalMarkerOffset(0.4,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .addTemporalMarker(()->{arm.setArmPos(0.50, 0.68);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{arm.setArmPos(0.7, 0.175);ArmV2.SliderLink(0.95);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, 0.175);})
                .resetConstraints()
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

        while (opModeInInit()) {
            slider.extendToHome();
            ArmV2.SetArmPosition(armServoOnePos, 0.16);
            Intake.SetArmPosition(0.5, 0.66);
            Intake.IntakePixel(0.8);
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

            //RIGHT TRAJECTORY
            switch (currentState){
                case Start:
                    if (gamepad1.b || propPosition == "RIGHT"){ //
                        if (!drive.isBusy()) {
                            currentState = AutoTrajectoryRight.AutoTrajectoryRightPurple;
                            drive.followTrajectorySequenceAsync(AutoTrajectoryRightPurple);
                        }
                    }
                    break;
                case AutoTrajectoryRightPurple:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.CenterPathPlacing;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing);
                    }
                    break;
                case CenterPathPlacing:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.AutoTrajectoryRightYellow;
                        drive.followTrajectorySequenceAsync(AutoTrajectoryRightYellow);
                    }
                    break;
                case AutoTrajectoryRightYellow:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.CenterPathPicking;
                        drive.followTrajectorySequenceAsync(CenterPathPicking);
                    }
                    break;
                case CenterPathPicking:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.CenterPathPlacing2;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing2);
                    }
                    break;
                case CenterPathPlacing2:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.AutoTrajectoryRightYellow2;
                        drive.followTrajectorySequenceAsync(AutoTrajectoryRightYellow2);
                    }
                    break;
                case AutoTrajectoryRightYellow2:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.ParkingOut;
//                        drive.followTrajectorySequenceAsync(ParkingOut);
                    }
                    break;
                case ParkingOut:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }


            //CENTER TRAJECTORY
            switch (currentState2){
                case Start:
                    if (gamepad1.y || propPosition == "CENTER"){ //
                        if (!drive.isBusy()) {
                            currentState2 = AutoTrajectoryCenter.AutoTrajectoryCenterPurple;
                            drive.followTrajectorySequenceAsync(AutoTrajectoryCenterPurple);
                        }
                    }
                    break;
                case AutoTrajectoryCenterPurple:
                    if (!drive.isBusy()) {
                        currentState2 = AutoTrajectoryCenter.CenterPathPlacing_Center;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing_Center);
                    }
                    break;
                case CenterPathPlacing_Center:
                    if (!drive.isBusy()) {
                        currentState2 = AutoTrajectoryCenter.AutoTrajectoryCenterYellow;
                        drive.followTrajectorySequenceAsync(AutoTrajectoryCenterYellow);
                    }
                    break;
                case AutoTrajectoryCenterYellow:
                    if (!drive.isBusy()) {
                        currentState2 = AutoTrajectoryCenter.CenterPathPicking_Center;
                        drive.followTrajectorySequenceAsync(CenterPathPicking_Center);
                    }
                    break;
                case CenterPathPicking_Center:
                    if (!drive.isBusy()) {
                        currentState2 = AutoTrajectoryCenter.CenterPathPlacing_Center2;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing_Center2);
                    }
                    break;
                case CenterPathPlacing_Center2:
                    if (!drive.isBusy()) {
                        currentState2 = AutoTrajectoryCenter.AutoTrajectoryCenterYellow2;
                        drive.followTrajectorySequenceAsync(AutoTrajectoryCenterYellow2);
                    }
                    break;
                case AutoTrajectoryCenterYellow2:
                    if (!drive.isBusy()) {
                        currentState2 = AutoTrajectoryCenter.ParkingOut;
//                        drive.followTrajectorySequenceAsync(ParkingOut);
                    }
                    break;
                case ParkingOut:
                    if (!drive.isBusy()) {
                        currentState2 = AutoTrajectoryCenter.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }


            //LEFT TRAJECTORY
            switch (currentState3){
                case Start:
                    if (gamepad1.x || propPosition == "LEFT"){ //
                        if (!drive.isBusy()) {
                            currentState3 = AutoTrajectoryLeft.AutoTrajectoryLeftPurple;
                            drive.followTrajectorySequenceAsync(AutoTrajectoryLeftPurple);
                        }
                    }
                    break;
                case AutoTrajectoryLeftPurple:
                    if (!drive.isBusy()) {
                        currentState3 = AutoTrajectoryLeft.CenterPathPlacing_Left;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing_Left);
                    }
                    break;
                case CenterPathPlacing_Left:
                    if (!drive.isBusy()) {
                        currentState3 = AutoTrajectoryLeft.AutoTrajectoryLeftYellow;
                        drive.followTrajectorySequenceAsync(AutoTrajectoryLeftYellow);
                    }
                    break;
                case AutoTrajectoryLeftYellow:
                    if (!drive.isBusy()) {
                        currentState3 = AutoTrajectoryLeft.CenterPathPicking_Left;
                        drive.followTrajectorySequenceAsync(CenterPathPicking_Left);
                    }
                    break;
                case CenterPathPicking_Left:
                    if (!drive.isBusy()) {
                        currentState3 = AutoTrajectoryLeft.CenterPathPlacing_Left2;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing_Left2);
                    }
                    break;
                case CenterPathPlacing_Left2:
                    if (!drive.isBusy()) {
                        currentState3 = AutoTrajectoryLeft.AutoTrajectoryLeftYellow2;
                        drive.followTrajectorySequenceAsync(AutoTrajectoryLeftYellow2);
                    }
                    break;
                case AutoTrajectoryLeftYellow2:
                    if (!drive.isBusy()) {
                        currentState3 = AutoTrajectoryLeft.ParkingOut;
//                        drive.followTrajectorySequenceAsync(ParkingOut);
//                        currentState3 = AutoTrajectoryLeft.IDLE;
                    }
                    break;
                case ParkingOut:
                    if (!drive.isBusy()) {
                        currentState3 = AutoTrajectoryLeft.IDLE;
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
