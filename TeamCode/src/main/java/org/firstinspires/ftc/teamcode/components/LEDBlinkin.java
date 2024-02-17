package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;

public class LEDBlinkin {
    public DcMotor blinkinPower;
    public RevBlinkinLedDriver lightRight, lightLeft;

    //constructor
    public LEDBlinkin(HardwareMap hardwareMap){
        //initialization
        lightLeft = hardwareMap.get(RevBlinkinLedDriver.class,"blinkinL");
        lightRight = hardwareMap.get(RevBlinkinLedDriver.class,"blinkinR");
        blinkinPower = hardwareMap.get(DcMotor.class,"blinkinPower");
    }

    public void turnLightOn(){
        blinkinPower.setPower(0.5);
    }

    public void turnLightOff(){
        blinkinPower.setPower(0.0);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern){
        lightLeft.setPattern(pattern);
        lightRight.setPattern(pattern);
    }
}
