package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmV2;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;

@TeleOp(group = "Robot Practice")
@Config
public class MatrixTest extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    ArmV2 arm = null;
    Hanger hanger = null;
    Intake intake = null;
    Drone drone = null;
    public static double THROTTLE = 1, HEADING = 1, TURN = 1;
    public static double armServoPos,armServoPos2, wristServoPos, deliveryServoPos, intakeArmServoPos, intakeWristServoPos, gripperServoPos, crankServoPos, armSliderServoPos, hangerServoPos1,hangerServoPos2,droneServoPos;

    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        drive = new SampleMecanumDrive(hardwareMap);
        slider = new Slider(hardwareMap, telemetry);
        arm = new ArmV2(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        drone =new Drone(hardwareMap, telemetry);

        AnalogInput armOneAnalogInput = hardwareMap.get(AnalogInput.class, "armOneAnalogInput");
        AnalogInput armTwoAnalogInput = hardwareMap.get(AnalogInput.class, "armTwoAnalogInput");
        AnalogInput intakeArmAnalogInput = hardwareMap.get(AnalogInput.class, "intakeArmAnalogInput");
        AnalogInput intakeWristAnalogInput = hardwareMap.get(AnalogInput.class, "intakeWristAnalogInput");
        AnalogInput crankAnalogInput = hardwareMap.get(AnalogInput.class, "crankAnalogInput");
        AnalogInput wristAnalogInput = hardwareMap.get(AnalogInput.class, "wristAnalogInput");

        DigitalChannel beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");
        beamBreaker.setMode(DigitalChannel.Mode.INPUT);

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

            // Main teleop loop goes here

            //drivetrain ---------------------------------------------------------------------------
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(Math.pow(Range.clip(-gamepad1.left_stick_y, -1, 1), 3),
                    Math.pow(Range.clip(-gamepad1.left_stick_x, -1, 1), 3)).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(input.getX() * THROTTLE, input.getY() * TURN, -gamepad1.right_stick_x * HEADING)
            );
            drive.update();
            telemetry.addData("heading", poseEstimate.getHeading());

            //Slider
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                Slider.IncreaseExtension(70);
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                Slider.DecreaseExtension(0);
            }
            //--------------------------------------------------------------------------------------

            //Arm
            if (currentGamepad1.x && !previousGamepad1.x){
                ArmV2.SetArm(armServoPos);
                ArmV2.wristServo.setPosition(wristServoPos);
            }
            if(currentGamepad1.start && !previousGamepad1.start){
                ArmV2.SliderLink(armSliderServoPos);
            }
            //Intake
            if(currentGamepad1.y && !previousGamepad1.y) {
                Intake.SetArmPosition(intakeArmServoPos, intakeWristServoPos);
            }
            //Gripper
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                Intake.IntakePixel(gripperServoPos);
            }
            //Crank
            if (currentGamepad1.b && !previousGamepad1.b){
                Intake.CrankPosition(crankServoPos);
            }
            //Delivery
            if(currentGamepad1.a && !previousGamepad1.a){
                ArmV2.DropPixel(deliveryServoPos);
            }
            //Hanger
            if(currentGamepad1.back && !previousGamepad1.back){
                Hanger.hangerServoOne.setPosition(hangerServoPos1);
                Hanger.hangerServoTwo.setPosition(hangerServoPos2);
            }
            if(currentGamepad2.y && !previousGamepad2.y) {
                Intake.intakeArmServo.setPosition(intakeArmPosition);
            }

//            if (currentGamepad1.dpad_up){
//                Hanger.LiftRobot();
//            }
//            if (currentGamepad1.dpad_down){
//                Hanger.PutDownRobot();
//            }
//            if (currentGamepad1.dpad_left){
//                Drone.droneServo.setPosition(droneServoPos);
//            }

            telemetry.addLine("Axon Positions");
            telemetry.addData("beam Breaker State", beamBreaker.getState());
            telemetry.addData("armTwoPosition", armTwoPosition);
            telemetry.addData("armOnePosition", armOnePosition);
            telemetry.addData("wristPosition", wristPosition);
            telemetry.addData("crankPosition", crankPosition);
            telemetry.addData("intakeWristPosition", intakeWristPosition);
            telemetry.addData("intakeArm Position", intakeArmPosition);

            telemetry.addLine("Servo Positions");
            telemetry.addData("gripperServo", Intake.gripperServo.getPosition());
            telemetry.addData("intakeWristServo", Intake.intakeWristServo.getPosition());
            telemetry.addData("intakeArmServo", Intake.intakeArmServo.getPosition());
            telemetry.addData("crankServo", Intake.crankServo.getPosition());
            telemetry.addData("armServoOne", ArmV2.armServoOne.getPosition());
            telemetry.addData("armServoTwo", ArmV2.armServoTwo.getPosition());
            telemetry.addData("wristServo", ArmV2.wristServo.getPosition());
            telemetry.addData("armSlider", ArmV2.armSliderServo.getPosition());
            telemetry.addData("deliveryServo", ArmV2.deliveryServo.getPosition());
            telemetry.addData("Hanger Servo One", Hanger.hangerServoOne.getPosition());
            telemetry.addData("Hanger Servo Two", Hanger.hangerServoTwo.getPosition());
            telemetry.addData("Drone Servo", Drone.droneServo.getPosition());
            telemetry.update();
        }
    }
}
