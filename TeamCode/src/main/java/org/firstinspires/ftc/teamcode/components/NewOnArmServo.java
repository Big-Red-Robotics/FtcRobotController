package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class NewOnArmServo {
    //initialize sensors & motors in public
    public Servo servo;

    //constructor
    public NewOnArmServo(HardwareMap hardwareMap){
        this.servo = hardwareMap.get(Servo.class, "ocServo");

        this.servo.setDirection(Servo.Direction.FORWARD);

    }

    public void left(){
        this.servo.setPosition(this.servo.getPosition()+0.1);
    }

    public void right() {
        this.servo.setPosition(this.servo.getPosition()-0.1);
    }

    //public (or private) methods
}
