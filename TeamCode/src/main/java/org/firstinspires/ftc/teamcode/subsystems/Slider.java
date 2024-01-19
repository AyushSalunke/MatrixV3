package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Slider {
    public static DcMotorEx
            sliderMotorOne = null, sliderMotorTwo = null;
    public static double motorPowerUP = 0.8, motorPowerDOWN = 0.8;
    public Slider(HardwareMap hardwareMap, Telemetry telemetry) {
        sliderMotorOne = hardwareMap.get(DcMotorEx.class, "sliderMotorOne");
        sliderMotorTwo = hardwareMap.get(DcMotorEx.class, "sliderMotorTwo");

        sliderMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sliderMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sliderMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sliderMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //motor directions
        sliderMotorOne.setDirection(DcMotorEx.Direction.FORWARD);
        sliderMotorTwo.setDirection(DcMotorEx.Direction.FORWARD);

        //reset motor encoders
        sliderMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    public static void IncreaseExtension(double level){
        sliderMotorOne.setTargetPosition((int) level);
        sliderMotorTwo.setTargetPosition((int) level);

        sliderMotorOne.setPower(motorPowerUP);
        sliderMotorTwo.setPower(motorPowerUP);

        sliderMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sliderMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public static void DecreaseExtension(double level){
        sliderMotorOne.setTargetPosition((int) level);
        sliderMotorTwo.setTargetPosition((int) level);

        sliderMotorOne.setPower(motorPowerDOWN);
        sliderMotorTwo.setPower(motorPowerDOWN);

        sliderMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sliderMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void extendTo(int targetPos,double pow){
        sliderMotorOne.setTargetPosition(targetPos);
        sliderMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sliderMotorOne.setPower(pow);

        sliderMotorTwo.setTargetPosition(targetPos);
        sliderMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sliderMotorTwo.setPower(pow);

    }

    public void extendToHome(){
        extendTo(0,0.8);
    }

}
