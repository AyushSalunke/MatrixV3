package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drone {
    public static Servo droneServo;
    public static double shootPosition = 0.3, initialPosition = 0.05;
    public Drone(HardwareMap hardwareMap, Telemetry telemetry) {
        droneServo = hardwareMap.get(Servo.class, "droneServo");
    }
    public static void shootDrone(){
        droneServo.setPosition(shootPosition);
    }
    public static void initialPos(){
        droneServo.setPosition(initialPosition);
    }
}
