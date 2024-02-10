package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;


//mahika
public class Drone {
    public Servo launchDrone;
    public Servo positionDrone;

    //constructor
    public Drone(HardwareMap hardwareMap){
        this.launchDrone = hardwareMap.get(Servo.class, RobotConfig.launchDrone);
        this.positionDrone = hardwareMap.get(Servo.class, RobotConfig.positionDrone);
        home();
    }

    public void home(){
        launchDrone.setPosition(0.6);
        positionDrone.setPosition(0.1);
    }

    public void prepareLaunch(){
        launchDrone.setPosition(0.6);
        positionDrone.setPosition(0.55);
    }

    public void launch(){
        launchDrone.setPosition(0.95);
        positionDrone.setPosition(0.55);
    }
}
