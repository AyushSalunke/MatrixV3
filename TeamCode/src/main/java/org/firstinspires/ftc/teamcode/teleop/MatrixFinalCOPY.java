package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.constants.TeleOpConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ArmV2;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "Robot Practice")
@Config
@Disabled
public class MatrixFinalCOPY extends LinearOpMode {
    SampleMecanumDrive drive = null;
    DriveTrain drivetrain = null;
    Slider slider = null;
    ArmV2 arm = null;
    Hanger hanger = null;
    Intake intake = null;
    Drone drone = null;
    TeleOpConstants.intakeConstants intakeConstants;
    public static DcMotorEx leftFront, leftRear, rightFront, rightRear;

    BNO055IMU imu;
    Orientation angles = new Orientation();
    double initYaw;
    double adjustedYaw;

    ElapsedTime inputTimer, outputTimer, angle_timer, dropTimer;
    public static double
            armServoOnePos = 0.92, armServoOneUP = 0.8, wristServoPos = 0.175, wristServoOutPos, deliveryServoPos, armSliderServoPos;
    public static double
            gripperServoPos, intakeArmServoPos, intakeWristServoPos, crankServoPos;
    public static int levelZero = 0, levelOne = 200, levelTwo = 400, levelThree = 500;
    boolean
            armToggle = false, intakeToggle = false, crankToggle = false,stackFlag = false, resetIntakeFlag = false;
    public static int intakeCounter, outtakeCounter,sliderCounter =0;
    public static double
            lifter_posL = 0, lifter_posR = 0, error_lifter, error_diff, error_int, error_lifterR, error_diffR, error_intR, errorprev, errorprevR, output_lifter, output_lifterR, output_power, target, dropVal;
    public static double
            error_servoOne, error_servoTwo, error_diffOne, error_diffTwo, error_prevOne, error_prevTwo, error_intOne, error_intTwo, output_servoOne, output_servoTwo;
    public static double kp = 4, ki, kd = 1.7;
    public static double fastSpeed = 1, slowSpeed = 0.5;
    public static double THROTTLE = 1, HEADING = 1, TURN = 1;
    public String hangerflag = "hangerDOWN";
//    private BHI260IMU imu;
    public enum IntakeState {
        INTAKE_GRIP_COMMAND,
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_GRIP,
        INTAKE_RETRACT,
        INTAKE_INPUT,
        INTAKE_FINAL
    };
    public enum OuttakeState{
        OUTTAKE_START,
        OUTTAKE_PUSH,
        OUTTAKE_OPEN,
        OUTTAKE_OUTPUT,
        OUTTAKE_SLIDER,
        OUTTAKE_FINAL
    };
    IntakeState inputState = IntakeState.INTAKE_START;
    OuttakeState outputState = OuttakeState.OUTTAKE_START;
    public static String intake_stack_command = "GroundIntake";

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        drive = new SampleMecanumDrive(hardwareMap);
        drivetrain = new DriveTrain(hardwareMap, telemetry);
        slider = new Slider(hardwareMap, telemetry);
        arm = new ArmV2(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        drone =new Drone(hardwareMap, telemetry);

        inputTimer = new ElapsedTime();
        outputTimer = new ElapsedTime();
        angle_timer = new ElapsedTime();
        dropTimer = new ElapsedTime();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        // Retrieve the IMU from the hardware map
//        IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
//        imu.initialize(parameters);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu1");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initYaw = angles.firstAngle;


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        AnalogInput intakeArmAnalogInput = hardwareMap.get(AnalogInput.class, "intakeArmAnalogInput");
        AnalogInput intakeWristAnalogInput = hardwareMap.get(AnalogInput.class, "intakeWristAnalogInput");
        AnalogInput crankAnalogInput = hardwareMap.get(AnalogInput.class, "crankAnalogInput");
        AnalogInput wristAnalogInput = hardwareMap.get(AnalogInput.class, "wristAnalogInput");
        AnalogInput armOneAnalogInput = hardwareMap.get(AnalogInput.class, "armOneAnalogInput");
        AnalogInput armTwoAnalogInput = hardwareMap.get(AnalogInput.class, "armTwoAnalogInput");

        DigitalChannel beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");
        beamBreaker.setMode(DigitalChannel.Mode.INPUT);

        drive.setPoseEstimate(PoseStorage.currentPose);

        while (opModeInInit()){
            ArmV2.SetArmPosition(armServoOnePos,wristServoPos);
            Intake.crankServo.setPosition(intakeConstants.crankRetractPos);
            Intake.intakeArmServo.setPosition(0.5);
            Intake.intakeWristServo.setPosition(0.66);
            ArmV2.DropPixel(0.95);
            Drone.initialPos();
            Hanger.hangerServoOne.setPosition(0.75);
            Hanger.hangerServoTwo.setPosition(0.25);
            Intake.gripperServo.setPosition(1);
            ArmV2.SliderLink(0.95);
            inputTimer.reset();
            outputTimer.reset();
            dropTimer.reset();
            intakeCounter = 0;
            sliderCounter = 0;
            outtakeCounter = 0;
        }

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double intakeArmPosition = intakeArmAnalogInput.getVoltage() / 3.3 * 360;
            double intakeWristPosition = intakeWristAnalogInput.getVoltage() / 3.3 * 360;
            double crankPosition = crankAnalogInput.getVoltage() / 3.3 * 360;
            double wristPosition = wristAnalogInput.getVoltage() / 3.3 * 360;
            double armOnePosition = armOneAnalogInput.getVoltage() / 3.3 * 360;
            double armTwoPosition = armTwoAnalogInput.getVoltage() / 3.3 * 360;

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            adjustedYaw = angles.firstAngle-initYaw;
            // toggle field/normal
            double zerodYaw = -initYaw+angles.firstAngle;

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double theta = Math.atan2(y, x) * 180/Math.PI; // aka angle
            double realTheta;
            realTheta = (360 - zerodYaw) + theta;
            double power = Math.hypot(x, y);
            double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
            double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
            double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

            double leftFrontPow = (power * cos / maxSinCos + turn);
            double rightFrontPow = (power * sin / maxSinCos - turn);
            double leftBackPow = (power * sin / maxSinCos + turn);
            double rightBackPow = (power * cos / maxSinCos - turn);


            if ((power + Math.abs(turn)) > 1) {
                leftFrontPow /= power + turn;
                rightFrontPow /= power - turn;
                leftBackPow /= power + turn;
                rightBackPow /= power - turn;
            }

            leftFront.setPower(leftFrontPow);
            rightFront.setPower(rightFrontPow);
            leftRear.setPower(leftBackPow);
            rightRear.setPower(rightBackPow);
            //--------------------------------------------------------------------------------------
//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;
//
//            if (currentGamepad1.start && !previousGamepad1.start) {
//                imu.resetYaw();
//            }
//
//            // Main teleop loop goes here
//
//            //drivetrain ---------------------------------------------------------------------------
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            rotX = rotX * 1.1;
//
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;
//
//            leftFront.setPower(fastSpeed * frontLeftPower);
//            leftRear.setPower(fastSpeed * backLeftPower);
//            rightFront.setPower(fastSpeed * frontRightPower);
//            rightRear.setPower(fastSpeed * backRightPower);
//
//            if (currentGamepad1.left_trigger > 0.75){
//                leftFront.setPower(slowSpeed * frontLeftPower);
//                leftRear.setPower(slowSpeed * backLeftPower);
//                rightFront.setPower(slowSpeed * frontRightPower);
//                rightRear.setPower(slowSpeed * backRightPower);
//            }
//
//            drive.update();
//
//            // Retrieve your pose
//            Pose2d myPose = drive.getPoseEstimate();
//
//            double turnStack = angleWrap(Math.toRadians(90) - botHeading);
//            double turnBackDrop = angleWrap(Math.toRadians(270) - botHeading);
//
//            double turnBotFront = angleWrap(Math.toRadians(0) - botHeading);
//            double turnBotBack = angleWrap(Math.toRadians(180) - botHeading);
//
//            //TODO: SPEED THIS UP, PID TUNING
//            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left ) {
//                drive.turn(turnStack);
//            }
//            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
//                drive.turn(turnBackDrop);
//            }
//            if (currentGamepad1.dpad_down && currentGamepad1.dpad_down ) {
//                drive.turn(turnBotBack);
//            }
//            if (currentGamepad1.dpad_up && currentGamepad1.dpad_up){
//                drive.turn(turnBotFront);
//            }
            //--------------------------------------------------------------------------------------
//            Pose2d poseEstimate = drive.getPoseEstimate();
//            Vector2d input = new Vector2d(Math.pow(Range.clip(gamepad1.left_stick_y, -1, 1), 3),
//                    Math.pow(Range.clip(gamepad1.left_stick_x, -1, 1), 3)).rotated(-poseEstimate.getHeading());
//
//            drive.setWeightedDrivePower(
//                    new Pose2d(input.getX() * THROTTLE, input.getY() * TURN, -gamepad1.right_stick_x * HEADING)
//            );
//            drive.update();
//            telemetry.addData("heading", poseEstimate.getHeading());

            //--------------------------------------------------------------------------------------

            //Intake Sequence
            switch (inputState) {
                case INTAKE_START:
                    if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && (intakeCounter == 0) && !crankToggle) {
                        ArmV2.wristServo.setPosition(wristServoPos);
                        ArmV2.SetArm(armServoOneUP);
                        ArmV2.DropPixel(1);
                        if (ArmV2.armServoOne.getPosition() == armServoOneUP && intake_stack_command == "GroundIntake"){
                            Intake.intakeArmServo.setPosition(0.4);
                            Intake.intakeWristServo.setPosition(0.495);
                            Intake.IntakePixel(1);
                            inputTimer.reset();
                            inputState = IntakeState.INTAKE_EXTEND;
                        }
                        if(intake_stack_command == "FiveStackGo") {
                            Intake.intakeArmServo.setPosition(0.650);
                            Intake.intakeWristServo.setPosition(0.23); //0.24
                            Intake.IntakePixel(1);
                            inputTimer.reset();
                            inputState = IntakeState.INTAKE_EXTEND;
                        }
                        if (intake_stack_command == "ThreeStackGo")
                        {
                            Intake.intakeArmServo.setPosition(0.520);
                            Intake.intakeWristServo.setPosition(0.365); //0.375
                            Intake.IntakePixel(1);
                            inputTimer.reset();
                            inputState = IntakeState.INTAKE_EXTEND;
                        }

                    }
                    if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && (intakeCounter == 2)) {
                        Intake.IntakePixel(1);
                        ArmV2.wristServo.setPosition(wristServoPos);
                        ArmV2.SetArm(armServoOneUP);
                        ArmV2.DropPixel(0.75);
                        if (ArmV2.armServoOne.getPosition() == armServoOneUP) {
                            Intake.intakeArmServo.setPosition(0.4);
                            Intake.intakeWristServo.setPosition(0.495);
                            Intake.IntakePixel(1);
                            inputTimer.reset();
                            inputState = IntakeState.INTAKE_EXTEND;
                        }
                    }
                    break;
                case INTAKE_EXTEND:
                    Intake.CrankPosition(0.45);
                    if (Intake.crankServo.getPosition() == 0.45) { //inputTimer.milliseconds() >= 200
                        inputTimer.reset();
                        inputState = IntakeState.INTAKE_GRIP;
                    }
                    break;
                case INTAKE_GRIP:
                    if (intake_stack_command == "GroundIntake")
                    {
                        Intake.intakeArmServo.setPosition(0.4);
                        Intake.intakeWristServo.setPosition(0.48);
                    }
                    if (intake_stack_command == "ThreeStackGo")
                    {
                        Intake.intakeArmServo.setPosition(0.53);
                        Intake.intakeWristServo.setPosition(0.365);//0.375
                    }
                    if (intake_stack_command == "FiveStackGo")
                    {
                        Intake.intakeArmServo.setPosition(0.650);
                        Intake.intakeWristServo.setPosition(0.23);//0.24
                    }
                    if (!intakeToggle) {
                        if (!beamBreaker.getState()) {
                            if (intake_stack_command == "GroundIntake")
                            {
                                Intake.intakeArmServo.setPosition(0.4);
                                Intake.intakeWristServo.setPosition(0.48);
                            }
                            if (intake_stack_command == "ThreeStackGo")
                            {
                                Intake.intakeArmServo.setPosition(0.53);
                                Intake.intakeWristServo.setPosition(0.365);//0.375
                            }
                            if (intake_stack_command == "FiveStackGo")
                            {
                                Intake.intakeArmServo.setPosition(0.650);
                                Intake.intakeWristServo.setPosition(0.23);//0.24
                            }
                            TrajectorySequence IntakePixel = drive.trajectorySequenceBuilder(startPose)
                                    .addTemporalMarker(() -> {
                                        Intake.CrankPosition(0.35);
                                    })
                                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                                        Intake.IntakePixel(0.8);
                                    })
                                    .waitSeconds(0.3)
                                    .build();
                            drive.followTrajectorySequence(IntakePixel);
                            drive.update();
                            if (inputTimer.milliseconds() >= 500) { ///800
                                inputTimer.reset();
                                inputState = IntakeState.INTAKE_RETRACT;
                            }
                        }
                    }
                    if (intakeToggle) {
                        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                            if (intake_stack_command == "GroundIntake")
                            {
                                Intake.intakeArmServo.setPosition(0.4);
                                Intake.intakeWristServo.setPosition(0.48);
                            }
                            if (intake_stack_command == "FiveStackGo")
                            {
                                Intake.intakeArmServo.setPosition(0.65);
                                Intake.intakeWristServo.setPosition(0.23);
                            }
                            if (intake_stack_command == "ThreeStackGo")
                            {
                                Intake.intakeArmServo.setPosition(0.53);
                                Intake.intakeWristServo.setPosition(0.365);
                            }
                            TrajectorySequence IntakePixel = drive.trajectorySequenceBuilder(startPose)
                                    .addTemporalMarker(() -> {
                                        Intake.CrankPosition(0.35);
                                    })
                                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                                        Intake.IntakePixel(0.8);
                                    })
                                    .waitSeconds(0.3)
                                    .build();
                            drive.followTrajectorySequence(IntakePixel);
                            drive.update();
                            if (inputTimer.milliseconds() >= 500) { // 800
                                inputTimer.reset();
                                inputState = IntakeState.INTAKE_RETRACT;
                            }
                        }
                    }
//                    if (beamBreaker.getState() && inputTimer.milliseconds() >= 5000) {
//                        TrajectorySequence CancelIntakePixel = drive.trajectorySequenceBuilder(startPose)
//                                .addTemporalMarker(() -> {
//                                    Intake.intakeArmServo.setPosition(0.5);
//                                    Intake.intakeWristServo.setPosition(0.66);
//                                })
//                                .waitSeconds(0.2)
//                                .addTemporalMarker(() -> {
//                                    Intake.CrankPosition(0.69);
//                                })
//                                .waitSeconds(0.3)
//                                .addTemporalMarker(() -> {
//                                    arm.setArmPos(armServoOnePos, wristServoPos);
//                                })
//                                .build();
//                        drive.followTrajectorySequence(CancelIntakePixel);
//                        intakeCounter = 0;
//                        intake_stack_command = "GroundIntake";
//                        inputState = IntakeState.INTAKE_START;
//                    }
                    if (resetIntakeFlag){
                        TrajectorySequence CancelIntakePixel = drive.trajectorySequenceBuilder(startPose)
                                .addTemporalMarker(() -> {Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                                .waitSeconds(0.2)
                                .addTemporalMarker(() -> {Intake.CrankPosition(0.69);})
                                .waitSeconds(0.3)
                                .addTemporalMarker(() -> {arm.setArmPos(armServoOnePos, wristServoPos);})
//                                .waitSeconds(0.3)
                                .build();
                        drive.followTrajectorySequence(CancelIntakePixel);
                        intakeCounter = 0;
                        resetIntakeFlag = false;
                        intake_stack_command = "GroundIntake";
                        inputState = IntakeState.INTAKE_START;
                    }
                    break;
                case INTAKE_RETRACT:
                    Intake.CrankPosition(0.69);
                    if (intake_stack_command == "FiveStackGo") {
                        Intake.intakeArmServo.setPosition(0.67);
                        Intake.intakeWristServo.setPosition(0.258);
                    }
                    if (intake_stack_command == "ThreeStackGo") {
                        Intake.intakeArmServo.setPosition(0.53);
                        Intake.intakeWristServo.setPosition(0.39);
                    }
                    if (Intake.crankServo.getPosition() == 0.69 && inputTimer.milliseconds()>=450){
                        Intake.intakeArmServo.setPosition(0.4);
                        Intake.intakeWristServo.setPosition(0.5);
                        if (inputTimer.milliseconds() >= 550 ) { //inputTimer.milliseconds() >= 300 // 500 //800
                            inputTimer.reset();
                            inputState = IntakeState.INTAKE_INPUT;
                        }
                    }
                    break;
                case INTAKE_INPUT:
                    Intake.intakeWristServo.setPosition(0.66);
                    Intake.intakeArmServo.setPosition(0.4);
                    Intake.IntakePixel(0.77);
                    if (intakeWristPosition>=130 && inputTimer.milliseconds() >= 400 ) { //inputTimer.milliseconds() >= 400
                        Intake.intakeArmServo.setPosition(0.75);
                        Intake.IntakePixel(0.77);
                        gamepad1.rumble(100);
                        if (intakeArmPosition <= 117 && inputTimer.milliseconds() >= 500) { //inputTimer.milliseconds() >= 500
                            Intake.intakeWristServo.setPosition(0.45);
                            Intake.intakeArmServo.setPosition(1);
                            Intake.IntakePixel(0.77);
                            Intake.crankServo.setPosition(0.69);
                            inputTimer.reset();
                            inputState = IntakeState.INTAKE_FINAL;
                        }
                    }
                    break;
                case INTAKE_FINAL:
                    ArmV2.wristServo.setPosition(wristServoPos);
                    if (Intake.intakeArmServo.getPosition() == 1 && inputTimer.milliseconds() >= 350 && wristPosition >= 280) {  //350 //inputTimer.milliseconds() >= 300
                        ArmV2.SetArmPosition(armServoOnePos, wristServoPos);
                        if (ArmV2.armServoOne.getPosition() == armServoOnePos && inputTimer.milliseconds() >= 500) { //350 //400
                            ArmV2.DropPixel(0.5);
                            ArmV2.SetArmPosition(0.92, wristServoPos);
                            output_power = lifter_pid(kp, ki, kd, -10);
                            if (output_power > 0.9) {
                                output_power = 1;
                            } else if (output_power < 0.2) {
                                output_power = 0;
                            }
                            slider.extendTo(-10, output_power);
                            Intake.IntakePixel(0.95);
                            if (inputTimer.milliseconds() >= 700) { // 350//500 //600
                                output_power = lifter_pid(kp, ki, kd, 0);
                                slider.extendTo(0, output_power);
                                Intake.IntakePixel(0.8);
                            }
                            ArmV2.SetArmPosition(armServoOnePos, wristServoPos);
                            if (stackFlag) {
                                intakeCounter = 2;
                                stackFlag = false;
                            } else {
                                intakeCounter = 1;
                            }
                            inputTimer.reset();
                            outtakeCounter = 0;
                            intake_stack_command = "GroundIntake";
                            inputState = IntakeState.INTAKE_START;
                        }
                    }
                    break;
                default:
                    inputState = IntakeState.INTAKE_START;
                    intakeCounter = 0;
            }
            //--------------------------------------------------------------------------------------

            //Outtake Sequence
            switch (outputState){
                case OUTTAKE_START:
                    //waiting for input
                    if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && (Intake.intakeArmServo.getPosition() > 0.75)){
                        outputTimer.reset();
                        outtakeCounter = 0;
                        outputState = OuttakeState.OUTTAKE_PUSH;
                    }
                    break;
                case OUTTAKE_PUSH:
                    Intake.intakeArmServo.setPosition(1);
                    Intake.intakeWristServo.setPosition(0.45);
                    Intake.crankServo.setPosition(0.69);
                    ArmV2.SetArmPosition(0.95, wristServoPos);
                    if (outputTimer.milliseconds() >= 200){ //200
                        ArmV2.DropPixel(0.5);
                        output_power = lifter_pid(kp, ki, kd, -10);
                        if (output_power > 0.9) {
                            output_power = 1;
                        } else if (output_power < 0.2) {
                            output_power = 0;
                        }
                        slider.extendTo(-10, output_power);
                        if (outputTimer.milliseconds() >= 300){//400
                            output_power = lifter_pid(kp, ki, kd, 0);
                            if (output_power > 0.9) {
                                output_power = 1;
                            } else if (output_power < 0.2) {
                                output_power = 0;
                            }
                            slider.extendTo(0, output_power);
                            ArmV2.SetArmPosition(armServoOnePos, wristServoPos);
                            outputTimer.reset();
                            outputState = OuttakeState.OUTTAKE_OPEN;
                        }
                    }
                    break;
                case OUTTAKE_OPEN:
                    Intake.IntakePixel(1);
                    if(outputTimer.milliseconds()>=200) {
                        Intake.intakeWristServo.setPosition(0.38);
                        if (outputTimer.milliseconds()>=400) {//400
                            Intake.intakeArmServo.setPosition(0.5);
                            Intake.intakeWristServo.setPosition(0.66);
                            outputTimer.reset();
                            outputState = OuttakeState.OUTTAKE_OUTPUT;
                        }
                    }
                    break;
                case OUTTAKE_OUTPUT:
                    ArmV2.SetArmPosition(0.5, wristServoPos);
                    if (outputTimer.milliseconds()>=400 ){ //300 && armTwoPosition >= 175
                        ArmV2.SetArmPosition(0.515, 0.66);
                    }
                    if (outputTimer.milliseconds() >= 500){ // 500
                        outputTimer.reset();
                        if (sliderCounter != 0) {
                            outputState = OuttakeState.OUTTAKE_SLIDER;
                        }
                        else {
                            outputState = OuttakeState.OUTTAKE_FINAL;
                        }

                    }
                    break;
                case OUTTAKE_SLIDER:
                    if (outputTimer.milliseconds() >= 200){ //800
                        if (sliderCounter == 1) {
                            output_power = lifter_pid(kp, ki, kd, levelOne);
                            if (output_power > 0.9) {
                                output_power = 1;
                            } else if (output_power < 0.2) {
                                output_power = 0;
                            }
                            slider.extendTo(levelOne, output_power);
                            outtakeCounter = 0;
                        }
                        if (sliderCounter == 2) {
                            output_power = lifter_pid(kp, ki, kd, levelTwo);
                            if (output_power > 0.9) {
                                output_power = 1;
                            } else if (output_power < 0.2) {
                                output_power = 0;
                            }
                            slider.extendTo(levelTwo, output_power);
                            outtakeCounter = 0;
                        }
                        if (sliderCounter == 3) {
                            output_power = lifter_pid(kp, ki, kd, levelThree);
                            if (output_power > 0.9) {
                                output_power = 1;
                            } else if (output_power < 0.2) {
                                output_power = 0;
                            }
                            slider.extendTo(levelThree, output_power);
                            outtakeCounter = 0;
                        }
                        outputTimer.reset();
                        outputState = OuttakeState.OUTTAKE_FINAL;
                    }
                    break;
                case OUTTAKE_FINAL:
                    Intake.crankServo.setPosition(0.69);
                    Intake.intakeArmServo.setPosition(0.5);
                    Intake.intakeWristServo.setPosition(0.67);
                    if (outputTimer.milliseconds()>=50){
                        outputTimer.reset();
                        intakeCounter = 0;
                        outtakeCounter = 1;
                        outputState = OuttakeState.OUTTAKE_START;
                    }
                    break;
                default:
                    outputState = OuttakeState.OUTTAKE_START;
            }

            if (outtakeCounter != 0 && (gamepad2.left_stick_x != 0 || gamepad2.left_stick_y != 0)){
                double armSliderValue = Range.clip(-gamepad2.left_stick_y,0,1);
                double mappedCrank = Range.scale(armSliderValue, 0, 1, 0.95, 0.2);

                ArmV2.armSliderServo.setPosition(mappedCrank);
            }
//            if (outtakeCounter != 0 && (gamepad2.right_stick_x != 0)){
//                double mappedYaw = Range.scale(gamepad2.left_stick_x, -1, 1, 0.52, 0.37);
//                ArmV2.armServoTwo.setPosition(mappedYaw);
//                ArmV2.armServoOne.setPosition(mappedYaw+0.1);
//            }
            //--------------------------------------------------------------------------------------

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && (intakeCounter == 1) && (Intake.intakeArmServo.getPosition() == 1)){
                intakeCounter = 0;
                TrajectorySequence ResetIntake = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{Intake.IntakePixel(1);slider.extendTo(0, 0.5);})
                        .waitSeconds(0.1)
                        .addTemporalMarker(()->{ArmV2.DropPixel(0.75);})
                        .waitSeconds(0.2)
                        .addTemporalMarker(()->{arm.setArmPos(0.8, wristServoPos);})
                        .waitSeconds(0.2)
                        .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.38);})
                        .waitSeconds(0.2)//0.4
                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                        .waitSeconds(0.2)
                        .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, wristServoPos);})
                        .build();
                drive.followTrajectorySequenceAsync(ResetIntake);
                intakeCounter = 2;
                drive.update();
            }
            if (currentGamepad1.right_trigger > 0.5 && !(previousGamepad1.right_trigger > 0.5) && !resetIntakeFlag){
                resetIntakeFlag = true;
            }

            if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper && (Intake.intakeArmServo.getPosition() < 0.75)){
                TrajectorySequence OuttakeArm = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{ArmV2.DropPixel(0.5);})
                        .addTemporalMarker(()->{arm.setArmPos(0.515, wristServoPos);})
                        .waitSeconds(0.4)
                        .addTemporalMarker(()->{arm.setArmPos(0.55, 0.66);})
                        .build();
                drive.followTrajectorySequenceAsync(OuttakeArm);
                drive.update();
            }

            if((currentGamepad1.y && !previousGamepad1.y) && (inputState!= IntakeState.INTAKE_START || outputState!= OuttakeState.OUTTAKE_START)){
                inputState = IntakeState.INTAKE_START;
                outputState = OuttakeState.OUTTAKE_START;
                TrajectorySequence ResetRobot = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{Intake.CrankPosition(0.38);})
                        .waitSeconds(0.3)
                        .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.66); Intake.intakeArmServo.setPosition(0.5);})
                        .waitSeconds(0.1)
                        .addTemporalMarker(()->{Intake.IntakePixel(1);})
                        .addTemporalMarker(()->{arm.setArmPos(0.8, wristServoPos);})
                        .waitSeconds(0.2)
                        .addTemporalMarker(()->{ArmV2.DropPixel(0.95);})
                        .waitSeconds(0.5)
                        .addTemporalMarker(()->{})
                        .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.65); Intake.intakeArmServo.setPosition(0.5);})
                        .addTemporalMarker(()->{Intake.IntakePixel(1);})
                        .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                        .waitSeconds(0.5)
                        .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, wristServoPos);})
                        .waitSeconds(0.5)
                        .build();
                drive.followTrajectorySequenceAsync(ResetRobot);
                drive.update();
            }

            if(currentGamepad1.b && !previousGamepad1.b){
                //drop 1st pixel
//                deliveryServoPos = 0.79;
                ArmV2.DropPixel(0.84);
                TrajectorySequence DropPixelOne = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{ArmV2.DropPixel(0.84);})
                        .waitSeconds(0.2)
//                        .addTemporalMarker(()->{arm.setArmPos(0.5, 0.66);}) //0.48
//                        .waitSeconds(0.2)
//                        .addTemporalMarker(()->{arm.setArmPos(0.5, 0.66);})
                        .build();
                drive.followTrajectorySequenceAsync(DropPixelOne);
                dropTimer.reset();
                drive.update();
            }

            if(currentGamepad1.a && !previousGamepad1.a){
                //drop 2nd pixel
                output_power = lifter_pid(kp, ki, kd, levelZero);
                if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }
                TrajectorySequence DropPixelTwo = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                        .waitSeconds(0.3)
                        .addTemporalMarker(()->{arm.setArmPos(0.57, 0.66); ArmV2.SliderLink(0.95);})
                        .waitSeconds(0.4)
                        .addTemporalMarker(()->{arm.setArmPos(0.7, wristServoPos);})
                        .waitSeconds(0.4)
                        .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, wristServoPos);})
                        .waitSeconds(0.2)
                        .addTemporalMarker(()->{slider.extendTo(levelZero, output_power);})
                        .build();
                drive.followTrajectorySequenceAsync(DropPixelTwo);
                drive.update();
                sliderCounter = 0;
                outtakeCounter = 0;
            }

            if (currentGamepad1.x && !previousGamepad1.x){
                Drone.shootDrone();
            }
            if(currentGamepad1.back && !previousGamepad1.back){
                Hanger.ExtendHanger();
                hangerflag = "hangerUP";
                Intake.SetArmPosition(0.75, 0.66);
            }


            //--------------------------------------------------------------------------------------

            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up){
                sliderCounter = 1;
                if (outputState == OuttakeState.OUTTAKE_START && outtakeCounter == 1){
                    outputTimer.reset();
                    outputState = OuttakeState.OUTTAKE_SLIDER;
                }
            }
            if(currentGamepad2.dpad_right && !previousGamepad2.dpad_right){
                sliderCounter = 2;
                if (outputState == OuttakeState.OUTTAKE_START && outtakeCounter == 1){
                    outputTimer.reset();
                    outputState = OuttakeState.OUTTAKE_SLIDER;
                }
            }
            if(currentGamepad2.dpad_down && !previousGamepad2.dpad_down){
                sliderCounter = 3;
                if (outputState == OuttakeState.OUTTAKE_START && outtakeCounter == 1) {
                    outputTimer.reset();
                    outputState = OuttakeState.OUTTAKE_SLIDER;
                }
            }
            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left){

            }
            if(currentGamepad2.a && !previousGamepad2.a){
                TrajectorySequence replunge = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{arm.setArmPos(0.82, 0.165);})
                        .waitSeconds(0.3)
                        .addTemporalMarker(()->{if (ArmV2.wristServo.getPosition()==0.165){arm.setArmPos(armServoOnePos, 0.165);}})
                        .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, 0.165);})
                        .build();
                drive.followTrajectorySequenceAsync(replunge);
            }

//            if(currentGamepad2.b && !previousGamepad2.b){
//                TrajectorySequence armReset = drive.trajectorySequenceBuilder(startPose)
//                        .addTemporalMarker(()->{Intake.crankServo.setPosition(0.69);})
//                        .addTemporalMarker(()->{arm.setArmPos(0.1, wristServoPos);})
//                        .addTemporalMarker(()->{slider.extendTo(-10, output_power);})
//                        .waitSeconds(0.2)
//                        .addTemporalMarker(()->{slider.extendTo(0, output_power);arm.setArmPos(armServoOnePos, wristServoPos);})
//                        .build();
//                drive.followTrajectorySequenceAsync(armReset);
//            }

            if (currentGamepad2.right_trigger>0.5 && !(previousGamepad2.right_trigger >0.5 )){
                crankToggle = !crankToggle;
                if (crankToggle) {
                    TrajectorySequence openCrank = drive.trajectorySequenceBuilder(startPose)
                            .addTemporalMarker(()->{arm.setArmPos(0.82, wristServoPos);})
                            .waitSeconds(0.3)
                            .addTemporalMarker(()->{Intake.crankServo.setPosition(0.35);})
                            .waitSeconds(0.3)
                            .build();
                    drive.followTrajectorySequenceAsync(openCrank);
                    inputState = IntakeState.INTAKE_START;
                }
                else
                {
                    TrajectorySequence closeCrank = drive.trajectorySequenceBuilder(startPose)
                            .addTemporalMarker(()->{arm.setArmPos(0.82, wristServoPos);})
                            .waitSeconds(0.3)
                            .addTemporalMarker(()->{Intake.crankServo.setPosition(0.69);})
                            .waitSeconds(0.3)
                            .addTemporalMarker(()->{arm.setArmPos(armServoOnePos, wristServoPos);})
                            .build();
                    drive.followTrajectorySequenceAsync(closeCrank);
                    inputState = IntakeState.INTAKE_START;
                }
            }

            if (currentGamepad2.start && !previousGamepad2.start){ //currentGamepad2.start
                intakeToggle = !intakeToggle;
            }

            if(currentGamepad2.x && previousGamepad2.x){
//                Intake.SetArmPosition(intakeArmServoPos, intakeWristServoPos);
            }
            if(currentGamepad2.y && previousGamepad2.y && hangerflag == "hangerUP"){
                Hanger.LiftRobot();
                hangerflag = "hangerDOWN";
            }
            if(currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
                intake_stack_command = "FiveStackGo";
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }
            if(currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                intake_stack_command = "ThreeStackGo";
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
            telemetry.addData("IntakeToggle", intakeToggle);
            telemetry.addData("sliderCounter", sliderCounter);
            telemetry.addData("stackFlag", stackFlag);
            telemetry.addData("IntakeCounter", intakeCounter);
            telemetry.addData("Beam Breaker State:", beamBreaker.getState());
            telemetry.addData("OuttakeCounter", outtakeCounter);
            telemetry.addData("Intake_stack_Command", intake_stack_command);

            telemetry.addData("SliderMotorOne tick count", Slider.sliderMotorOne.getCurrentPosition());
            telemetry.addData("SliderMotorTwo tick count", Slider.sliderMotorTwo.getCurrentPosition());
            telemetry.addData("SliderMotorOne Current", Slider.sliderMotorOne.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("SliderMotorTwo Current", Slider.sliderMotorTwo.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("HangerMotor tick count", Hanger.hangerMotor.getCurrentPosition());
            telemetry.addData("Hanger Current", Hanger.hangerMotor.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("armOnePosition", armOnePosition);
            telemetry.addData("armTwoPosition", armTwoPosition);
            telemetry.addData("wristPosition", wristPosition);
            telemetry.addData("intakeWristPosition", intakeWristPosition);
            telemetry.addData("crankPosition", crankPosition);
            telemetry.addData("intakeArm Position", intakeArmPosition);
            telemetry.addData("gripperServo", Intake.gripperServo.getPosition());
            telemetry.addData("intakeWristServo", Intake.intakeWristServo.getPosition());
            telemetry.addData("intakeArmServo", Intake.intakeArmServo.getPosition());
            telemetry.addData("crankServo", Intake.crankServo.getPosition());
            telemetry.addData("armServoOne", ArmV2.armServoOne.getPosition());
            telemetry.addData("armServoTwo", ArmV2.armServoTwo.getPosition());
            telemetry.addData("wristServo", ArmV2.wristServo.getPosition());
            telemetry.addData("armSlider", ArmV2.armSliderServo.getPosition());
            telemetry.addData("deliveryServo", ArmV2.deliveryServo.getPosition());

            telemetry.addData("LeftFrontCurrent", DriveTrain.leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RightFrontCurrent", DriveTrain.rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("LeftRearCurrent", DriveTrain.leftRear.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RightRearCurrent", DriveTrain.rightRear.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
            drive.update();
        }
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
    public List<Double> servo_pid(double kp_servo, double ki_servo, double kd_servo, double targetOne, double targetTwo, double armOnePosition, double armTwoPosition)
    {
        error_servoOne = targetOne - armOnePosition;
        error_diffOne = error_servoOne - error_prevOne;
        error_intOne = error_servoOne + error_prevOne;
        output_servoOne = kp_servo * error_servoOne + kd_servo * error_diffOne + ki_servo * error_intOne;

        error_servoTwo = targetTwo - armTwoPosition;
        error_diffTwo = error_servoTwo - error_prevTwo;
        error_intTwo = error_servoTwo + error_prevTwo;
        output_servoTwo = kp_servo * error_servoTwo + kd_servo * error_diffTwo + ki_servo * error_intTwo;

        error_prevOne = error_servoOne;
        error_prevTwo = error_servoTwo;

        List<Double> ArmPosition = new ArrayList<>();
        ArmPosition.add(output_servoOne);
        ArmPosition.add(output_servoTwo);
        return ArmPosition;
    }
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }
    public static double joystickScalar(double num, double min) {
        return joystickScalar(num, min, 0.66, 4);
    }
    private static double joystickScalar(double n, double m, double l, double a) {
        return Math.signum(n) * m
                + (1 - m) *
                (Math.abs(n) > l ?
                        Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n) :
                        n / a);
    }

    float mapCubic(float x, float inMin, float inMax, float outMin, float outMax) {
        float normalizedX = (x - inMin) / (inMax - inMin);
        float mappedValue = normalizedX * normalizedX * normalizedX * (outMax - outMin) + outMin;
        return mappedValue;
    }
}