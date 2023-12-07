package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;

public class NewClaw {
    //no point of setting a limiter because there are hardwares that stops it. we can also use the claw.scaleRange to limit our movement.
    // TODO add a limiter if u want but doest it really matter. I might break stuff if a limiter isnt present but i feel like it will be fine.

    public Servo claw;

    public NewClaw(HardwareMap hardwareMap){
        this.claw = hardwareMap.get(Servo.class, RobotConfig.claw);
        this.claw.setDirection(Servo.Direction.FORWARD);
    }

    public double getPos(){
        return this.claw.getPosition();
    }

    public void close(){
        this.claw.setPosition(0.46);
    }

    public void open(){
        this.claw.setPosition(0.6);
    }

}
