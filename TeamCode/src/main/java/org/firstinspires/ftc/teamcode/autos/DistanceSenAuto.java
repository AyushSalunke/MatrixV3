package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmV2;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class DistanceSenAuto extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    ArmV2 arm = null;
    Hanger hanger = null;
    Intake intake = null;
    private DistanceSensor sensorDistance, sensorDistance2, sensorDistance3;
    enum Traj {
        Start,
        first,
        poseCorrection,
        IDLE
    }
    Traj currentState = Traj.Start;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensor_distance2");
        sensorDistance3 = hardwareMap.get(DistanceSensor.class, "sensor_distance3");

        Pose2d startPose=new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence first = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(15))
                .back(24)
                .resetConstraints()
                .build();

        while (opModeInInit()){
        }

        double errorPose = 0;

        waitForStart();

        while(opModeIsActive()){
            double reqDist = 5;
            double avgPos = ((sensorDistance.getDistance(DistanceUnit.INCH) + sensorDistance2.getDistance(DistanceUnit.INCH) + sensorDistance3.getDistance(DistanceUnit.INCH) / 3));
            double error = avgPos - reqDist;


            double factor = 1.17;

            double x = drive.getPoseEstimate().getX() + (error * factor);
            double y = drive.getPoseEstimate().getY();
            double heading = drive.getPoseEstimate().getHeading();

            switch (currentState){
                case Start:
                    if (gamepad1.y){
                        if(!drive.isBusy()){
                            drive.followTrajectorySequenceAsync(first);
                            currentState = Traj.poseCorrection;
                        }
                    }
                    break;
                case poseCorrection:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .back(error * factor)
                                .build());
                        errorPose = error*factor;
                        currentState = Traj.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            telemetry.addData("pose", drive.getPoseEstimate());

            telemetry.addData("error", error);
            telemetry.addData("errorPose", errorPose);

            telemetry.addData("range1", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range1", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));

            telemetry.addData("range2", String.format("%.01f mm", sensorDistance2.getDistance(DistanceUnit.MM)));
            telemetry.addData("range2", String.format("%.01f in", sensorDistance2.getDistance(DistanceUnit.INCH)));

            telemetry.addData("range3", String.format("%.01f mm", sensorDistance3.getDistance(DistanceUnit.MM)));
            telemetry.addData("range3", String.format("%.01f in", sensorDistance3.getDistance(DistanceUnit.INCH)));
            drive.update();
            telemetry.update();
        }
    }
}