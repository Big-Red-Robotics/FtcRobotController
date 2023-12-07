package org.firstinspires.ftc.teamcode.components;

import java.lang.String;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;

public class NewLift {
    //initialize sensors & motors in public
    public DcMotor lift1;
    public DcMotor lift2;
    /*
    1. If it operats and encoders know it is in a set range of values, we do not have to force start bottom. (This happens especially to changes of motor position after turning off. If the motor reads at 900 prior to closing. And you turn it off and pushes the motor or lift down. Although the lift is at the bottom it still reads 900, meaning it will violate the limits.
     */
    boolean isSimpleMode;

    public int liftMax = RobotConfig.liftMaxEncoder;

    public int liftMin = RobotConfig.liftMinEncoder;

    public double speed = 0.1;
    //constructor
    public NewLift(HardwareMap hardwareMap, boolean isSimpleMode, double speed){
        //initialization
        this.lift1 = hardwareMap.get(DcMotor.class, RobotConfig.lift);
        this.lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.lift1.setDirection(DcMotorSimple.Direction.FORWARD);

        this.lift2 = hardwareMap.get(DcMotor.class, RobotConfig.lift2);
        this.lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.lift2.setDirection(DcMotorSimple.Direction.REVERSE);
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
        return this.lift1.getCurrentPosition();
    }

    public boolean isExceedingLimit(){
        int curPos = getCurrentPosition();
        if(curPos <= this.liftMin + RobotConfig.liftBuffer || curPos >= this.liftMax - RobotConfig.liftBuffer)
            return true;

        return false;
    }

    public void up(){
        this.lift1.setPower(this.speed);
        this.lift2.setPower(this.speed);
    }

    public void down(){
        this.lift1.setPower(-this.speed);
        this.lift2.setPower(-this.speed);
    }

    public void stop(){this.lift1.setPower(0);this.lift2.setPower(0);}


    //public (or private) methods
}
