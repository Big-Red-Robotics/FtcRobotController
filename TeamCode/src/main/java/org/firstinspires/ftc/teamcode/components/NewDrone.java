package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;

import java.util.Arrays;
import java.util.List;

//mahika
public class NewDrone {

    private List<DcMotor> arms;
    //initialize sensors & motors in public
    public Servo launchDrone;
    public Servo positionDrone;

    //enum for state (servo 1: home, launchPos; servo 2: firstReleasePos, secondReleasePos)
    public enum droneServo1state {home,launchPos, none};
    public enum droneServo2state {firstReleasePos,secondReleasePos, none};

    public droneServo1state currentState  = droneServo1state.none;
    public droneServo2state currentState2  = droneServo2state.none;

    //constructor
    public NewDrone(HardwareMap hardwareMap){
        this.launchDrone = hardwareMap.get(Servo.class, RobotConfig.launchDrone);
        this.positionDrone = hardwareMap.get(Servo.class, RobotConfig.positionDrone);

    }

    //public (or private) methods
    public void homePosition() {}
    public void launchPosition(){}
    public void firstRelease() {}
    public void secondRelease() {}



    public void runToPosition(int position) {}

    public int getCurrentPosition(){return 0;}
    public int getTargetPosition(){return 0;}
    public double getPower(){return 0.0;}

}
