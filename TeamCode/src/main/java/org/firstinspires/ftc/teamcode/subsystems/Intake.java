package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    public static Servo gripperServo, intakeWristServo, intakeArmServo, crankServo;
    public static double
            gripperServoPos = 0.0, intakeArmServoPos = 0.0, intakeWristServoPos = 0.0, crankServoPos = 0.0;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
        intakeWristServo = hardwareMap.get(Servo.class, "intakeWristServo");
        intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServo");
        crankServo = hardwareMap.get(Servo.class, "crankServo");
    }
    public static void SetArmPosition(double intakeArmServoPos, double intakeWristServoPos) throws InterruptedException {
        intakeWristServo.setPosition(intakeWristServoPos);
//        sleep(100);
        intakeArmServo.setPosition(intakeArmServoPos);
    }
    public void setArm(double intakeArmServoPos, double intakeWristServoPos){
        intakeWristServo.setPosition(intakeWristServoPos);
        intakeArmServo.setPosition(intakeArmServoPos);
    }
    public static void IntakePixel(double gripperServoPos){
        gripperServo.setPosition(gripperServoPos);
    }
    public static void CrankPosition(double crankServoPos){
        crankServo.setPosition(crankServoPos);
    }
}
