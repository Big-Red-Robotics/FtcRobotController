package org.firstinspires.ftc.teamcode.components;

import java.lang.String;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;

public class NewLift {
    //initialize sensors & motors in public
    public DcMotor lift;
    /*
    1. If it operats and encoders know it is in a set range of values, we do not have to force start bottom. (This happens especially to changes of motor position after turning off. If the motor reads at 900 prior to closing. And you turn it off and pushes the motor or lift down. Although the lift is at the bottom it still reads 900, meaning it will violate the limits.
     */
    boolean isSimpleMode;

    public int liftMax = RobotConfig.liftMaxEncoder;

    public int liftMin = RobotConfig.liftMinEncoder;

    public double speed = 0.5;
    //constructor
    public NewLift(HardwareMap hardwareMap, boolean isSimpleMode, double speed){
        //initialization
        this.lift = hardwareMap.get(DcMotor.class, RobotConfig.lift);
        this.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.lift.setDirection(DcMotorSimple.Direction.FORWARD);
        if(!isSimpleMode)
            setUpComplexMode();

        this.speed = speed;
        this.isSimpleMode = isSimpleMode;

        // Assuming the encoders only reads the current position at
    }

    public void setUpComplexMode(){
        // This assumes that the slide position is at the bottom.
        this.liftMin = getCurrentPosition();
        this.liftMax = this.liftMin+RobotConfig.liftDifference;
    }

    public int getCurrentPosition(){
        return this.lift.getCurrentPosition();
    }

    public boolean isExceedingLimit(){
        int curPos = getCurrentPosition();
        if(curPos < this.liftMin + RobotConfig.liftBuffer || curPos > this.liftMax - RobotConfig.liftBuffer)
            return true;
        return false;
    }

    public void up(){
        this.lift.setPower(this.speed);
    }

    public void down(){this.lift.setPower(-this.speed);}

    public void stop(){this.lift.setPower(0);}


    //public (or private) methods
}
