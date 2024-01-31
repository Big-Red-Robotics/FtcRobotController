package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;

import java.util.Arrays;
import java.util.List;

public class Arm {
    private final DcMotor leftLift, rightLift;
    private final Servo clawRotator;
    private final ServoImplEx clawPivot;
    private final Servo leftClaw, rightClaw;
    private DcMotor armExtension;
    List<DcMotor> lifts;

    //arm position
    public final int GROUND = 0,
                     LOW = 410,
                     MIDDLE = 550,
                     HANG = 1100,
                     HIGH = 1380;

    private int rotatorLevel = 0;
    private boolean clawFlip = false;
    public boolean rightClawOpen = false, leftClawOpen = false;

    public Arm(HardwareMap hardwareMap){
        this.leftLift = hardwareMap.get(DcMotor.class, RobotConfig.liftL);
        this.rightLift = hardwareMap.get(DcMotor.class, RobotConfig.liftR);
        this.clawRotator = hardwareMap.get(Servo.class, RobotConfig.clawRotator);
        this.leftClaw = hardwareMap.get(Servo.class, RobotConfig.leftClaw);
        this.rightClaw = hardwareMap.get(Servo.class, RobotConfig.rightClaw);
        this.clawPivot = hardwareMap.get(ServoImplEx.class, RobotConfig.clawPivot);
        this.armExtension = hardwareMap.get(DcMotor.class, RobotConfig.armExtension);

        lifts = Arrays.asList(leftLift, rightLift);
        for(DcMotor lift: lifts){
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        rightLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setDirection(DcMotor.Direction.REVERSE);

        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtension.setTargetPosition(0);
        armExtension.setPower(0.0);

        clawPivot.setPwmRange(new PwmControl.PwmRange(1200, 1800));
        clawPivot.setPosition(0.95);
    }

    public void resetLift(){
        for(DcMotor lift: lifts){
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setTargetPosition(lift.getTargetPosition());
        }
    }

    public void resetArmExtension(){
        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtension.setTargetPosition(armExtension.getTargetPosition());
    }

    public void setArmExtensionPower(double power){
        armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtension.setPower(power);
        armExtension.setTargetPosition(armExtension.getCurrentPosition());
    }

    public void setLiftPosition(int armPosition){
        for(DcMotor lift: lifts){
            lift.setTargetPosition(armPosition);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void toggleClawRotator(){
        if(rotatorLevel == 2) rotatorLevel = 0;
        else rotatorLevel = 2;
    }

    public void setRotatorLevel(int level){
        rotatorLevel = level;
    }

    public void setClawFlip(boolean flip){
        this.clawFlip = flip;
    }

    //armEx
    public void setArmExtensionPosition(int position){
        armExtension.setTargetPosition(position);
        armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    //claw
    public void openLeftClaw() {
        leftClaw.setPosition(0.9);
        leftClawOpen = true;
    }
    public void closeLeftClaw() {
        leftClaw.setPosition(0.5);
        leftClawOpen = false;
    }
    public void toggleLeftClaw(){
        if(leftClawOpen) closeLeftClaw();
        else openLeftClaw();
    }

    public void openRightClaw() {
        rightClaw.setPosition(0.35);
        rightClawOpen = true;
    }
    public void closeRightClaw() {
        rightClaw.setPosition(0.9);
        rightClawOpen = false;
    }
    public void toggleRightClaw(){
        if(rightClawOpen) closeRightClaw();
        else openRightClaw();
    }

    public void toggleClaw(){
        if(rightClawOpen || leftClawOpen) closeClaw();
        else openClaw();
    }
    public void openClaw() {
        openLeftClaw();
        openRightClaw();
    }
    public void closeClaw() {
        closeLeftClaw();
        closeRightClaw();
    }

    //claw rotator
    public void setClawRotatorPosition(double position){
        clawRotator.setPosition(position);
    }

    public void setLiftPower(double power) {
        for (DcMotor lift: lifts) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setPower(power);
            lift.setTargetPosition(leftLift.getCurrentPosition());
        }
    }

    public void toPosition(int position, int rotator, boolean pivot){
        //claw pivot
        if (pivot) clawPivot.setPosition(0.91);
        else clawPivot.setPosition(0.0);

        //set lift target
        setLiftPosition(position);

        //set lift power
        while (leftLift.isBusy() || rightLift.isBusy()) {
            for(DcMotor lift: lifts){
                if (lift.getCurrentPosition() > lift.getTargetPosition()) {
                    if ((lift.getCurrentPosition() < 700 || lift.getCurrentPosition() > 1300) && lift.getTargetPosition() != HANG) {
                        //can depend on gravity
                        if (lift.getCurrentPosition() < MIDDLE) lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        else lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        lift.setPower(0.0);
                    } else lift.setPower(0.3);
                } else if(lift.getCurrentPosition() > LOW && lift.getTargetPosition() == MIDDLE) lift.setPower(0.3);
                else lift.setPower(0.8);
            }
        }

        //claw rotator
        this.rotatorLevel = rotator;
        if(rotator == 0) clawRotator.setPosition(0.0);
        else if (rotator == 1) clawRotator.setPosition(0.43);
        else if (rotator == 2) clawRotator.setPosition(0.85);
        else if (rotator == 3) clawRotator.setPosition(1);
        else if (rotator == 5) clawRotator.setPosition(0.1);
    }

    public void update(boolean rotateClaw) {
        //claw rotator
        if(rotateClaw){
            //ground: 0, low: 1, middle & high: 2, all the way: 3
            if(rotatorLevel == 0) clawRotator.setPosition(0.45);
            else if (rotatorLevel == 1) clawRotator.setPosition(0.6);
            else if (rotatorLevel == 2) clawRotator.setPosition(1);
            else if (rotatorLevel == 5) clawRotator.setPosition(0.5);
            else clawRotator.setPosition(1);
        }

        if (clawFlip) clawPivot.setPosition(0.29);
        else clawPivot.setPosition(0.95);

        if(armExtension.isBusy()) {
            if(getLiftTargetPosition() == LOW || getLiftTargetPosition() == MIDDLE) armExtension.setPower(0.6);
            else armExtension.setPower(1.0);
        }
        else armExtension.setPower(0.0);

        //the actual lift part
        for (DcMotor lift : lifts) {
            if (lift.isBusy()) {
                if (lift.getCurrentPosition() > lift.getTargetPosition()) {
                    if (lift.getCurrentPosition() < 700 && lift.getTargetPosition() != HANG) {
                        //can depend on gravity
                        if (lift.getCurrentPosition() < MIDDLE) lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        else lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        lift.setPower(0.0);
                    } else lift.setPower(0.4);
                } else if(lift.getCurrentPosition() > LOW && lift.getTargetPosition() == MIDDLE) lift.setPower(0.3);
                else lift.setPower(0.8);
            } else if (lift.getTargetPosition() == HIGH || lift.getTargetPosition() == GROUND) {
                lift.setPower(0.0);
            }
        }
    }

    public int getLiftPosition() {return (leftLift.getCurrentPosition() + rightLift.getCurrentPosition())/2;}
    public int getLiftTargetPosition() {return (leftLift.getTargetPosition() + rightLift.getTargetPosition())/2;}

    public int getRotatorLevel() {return rotatorLevel;}
    public double getRotatorPosition() {return clawRotator.getPosition();}
    public double getClawPivotPosition() {return clawPivot.getPosition();}
    public double getLeftClawPosition() {return leftClaw.getPosition();}
    public double getRightClawPosition() {return rightClaw.getPosition();}

    public boolean getClawFlip() {return clawFlip;}

    public double getArmExPower() {return armExtension.getPower();}
    public int getArmExPosition() {return armExtension.getCurrentPosition();}
    public int getArmExTargetPosition() {return armExtension.getTargetPosition();}
}
