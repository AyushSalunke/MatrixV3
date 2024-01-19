package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    public static Servo armServo, wristServo, deliveryServo;



    public static double
            armServoPos = 0.0, wristServoPos = 0.0, deliveryServoPos = 0.0;


    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        armServo = hardwareMap.get(Servo.class, "armServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        deliveryServo = hardwareMap.get(Servo.class, "deliveryServo");

    }
    public static void SetArmPosition(double armServoPos, double wristServoPos) throws InterruptedException {
        wristServo.setPosition(wristServoPos);
        sleep(100);
        armServo.setPosition(armServoPos);
    }
    public static void DropPixel(double deliveryServoPos){
        deliveryServo.setPosition(deliveryServoPos);
    }
    public static void SetPower(double ServoPower){}
}
