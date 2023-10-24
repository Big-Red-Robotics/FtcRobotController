package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;


//mahika
public class NewDrone {
    public Servo launchDrone;
    public Servo positionDrone;

    //constructor
    public NewDrone(HardwareMap hardwareMap){
        this.launchDrone = hardwareMap.get(Servo.class, RobotConfig.launchDrone);
        this.positionDrone = hardwareMap.get(Servo.class, RobotConfig.positionDrone);
        launchDrone.setPosition(0.65);
        positionDrone.setPosition(0.25);
    }

    public void home(){
        launchDrone.setPosition(0.65);
        positionDrone.setPosition(0.25);
    }

    public void prepareLaunch(){
        launchDrone.setPosition(0.65);
        positionDrone.setPosition(0.5);
    }

    public void launch(){
        launchDrone.setPosition(0.85);
        positionDrone.setPosition(0.5);
    }
}
