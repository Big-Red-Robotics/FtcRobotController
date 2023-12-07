package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;

public class NewArm {
    //initialize sensors & motors in public
    private DcMotor newArm;

    private final static int forwardLimit = 0;
    private final static int backwardLimit = 0;

    private static double speed;

    //constructor
    public NewArm(HardwareMap hardwareMap, double speed){
        this.speed = speed;
        this.newArm = hardwareMap.get(DcMotor.class, RobotConfig.NewArm);
        this.newArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.newArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.newArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //initialization
    }

    public void run(){
        this.newArm.setPower(speed);
    }

    public void stop(){
        this.newArm.setPower(0);
    }

    public int getPos(){
        return this.newArm.getCurrentPosition();
    }

    public void back(){
        this.newArm.setPower(-speed);
    }


}
