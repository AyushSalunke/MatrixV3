package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hanger {
    public static DcMotorEx hangerMotor;
    public static Servo hangerServoOne, hangerServoTwo, hangerServo;
    public static double motorPower = 0.8;
    public static double extendPosition = 1.0;
    public Hanger(HardwareMap hardwareMap, Telemetry telemetry) {

        hangerServoOne = hardwareMap.get(Servo.class, "hangerServoOne");
        hangerServoTwo = hardwareMap.get(Servo.class, "hangerServoTwo");

        hangerMotor = hardwareMap.get(DcMotorEx.class, "hangerMotor");
        hangerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hangerMotor.setDirection(DcMotorEx.Direction.REVERSE);
        hangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public static void ExtendHanger(){
        hangerServoOne.setPosition(1 - extendPosition);
        hangerServoTwo.setPosition(extendPosition);
    }
    public static void ExtendToHanger(double hangerPos){
        hangerServoOne.setPosition(1 - hangerPos);
        hangerServoTwo.setPosition(hangerPos);
    }
    public static void LiftRobot(){
        hangerMotor.setTargetPosition(hangerMotor.getCurrentPosition() + 3000 );
        hangerMotor.setPower(motorPower);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public static void PutDownRobot(){
        hangerMotor.setTargetPosition(hangerMotor.getCurrentPosition() - 3000 );
//        hangerMotor.setTargetPosition(hangerMotor.getCurrentPosition() - 4000 );
        hangerMotor.setPower(motorPower);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public static void HangerINC(){
        hangerMotor.setTargetPosition(hangerMotor.getCurrentPosition() + 200 );
        hangerMotor.setPower(motorPower);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public static void HangerDEC(){
        hangerMotor.setTargetPosition(hangerMotor.getCurrentPosition() - 200 );
        hangerMotor.setPower(motorPower);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
//    public static void StopHanger(){
//        hangerMotor.setPower(0);
//    }
}
