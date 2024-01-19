package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmV2 {
    public static Servo armServoOne, armServoTwo, wristServo, deliveryServo, armSliderServo;
    public static double
            armServoOnePos = 0.0, armServoTwoPos = 0.0, wristServoPos = 0.0, deliveryServoPos = 0.0, armSliderServoPos = 0.0;
    public ArmV2(HardwareMap hardwareMap, Telemetry telemetry) {
        armServoOne = hardwareMap.get(Servo.class, "armServoOne");
        armServoTwo = hardwareMap.get(Servo.class, "armServoTwo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        deliveryServo = hardwareMap.get(Servo.class, "deliveryServo");
        armSliderServo = hardwareMap.get(Servo.class, "armSliderServo");

    }
    public static void SetArmPosition(double armServoOnePos, double wristServoPos) throws InterruptedException {
        wristServo.setPosition(wristServoPos);
        armServoOne.setPosition(armServoOnePos);
        armServoTwo.setPosition(1 - armServoOnePos);

    }
    public static void SetWiperPositionRight(){
        armServoOne.setPosition(armServoOne.getPosition() + 0.1);
        armServoTwo.setPosition(armServoTwo.getPosition() + 0.1);
    }
    public static void SetWiperPositionLeft(){
        armServoOne.setPosition(armServoOne.getPosition() - 0.1);
        armServoTwo.setPosition(armServoTwo.getPosition() - 0.1);
    }
    public void setArmPos(double armServoOnePos, double wristServoPos) {
        wristServo.setPosition(wristServoPos);
        if(wristServo.getPosition() == wristServoPos){
            armServoOne.setPosition(armServoOnePos);
            armServoTwo.setPosition(1-armServoOnePos);
        }
    }
    public static void DropPixel(double deliveryServoPos){
        deliveryServo.setPosition(deliveryServoPos);
    }
    public static void SliderLink(double armSliderServoPos){
        armSliderServo.setPosition(armSliderServoPos);
    }
    public static void SetPower(double ServoPower){}
    public static void SetArm(double armServoOnePos) throws InterruptedException {
            armServoOne.setPosition(armServoOnePos);
            armServoTwo.setPosition(1 - armServoOnePos);
    }

}