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
    public final Servo clawRotator;
    public final ServoImplEx clawPivot;
    public final Servo leftClaw, rightClaw;
    public DcMotor armExtension;
    List<DcMotor> lifts;

    //arm position
    public int ground = 0,
               low_b = 410,
               low = 350,
               middle = 410,
               hang = 1100,
               high = 1380;

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
    public void setArmExtensionPower(double power){
        armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtension.setPower(power);
        armExtension.setTargetPosition(armExtension.getCurrentPosition());
    }

    //claw
    public void openLeftClaw() {
        leftClaw.setPosition(0.4);
        leftClawOpen = true;
    }
    public void closeLeftClaw() {
        leftClaw.setPosition(0.75);
        leftClawOpen = false;
    }
    public void toggleLeftClaw(){
        if(leftClawOpen) closeLeftClaw();
        else openLeftClaw();
    }

    public void openRightClaw() {
        rightClaw.setPosition(0.7);
        rightClawOpen = true;
    }
    public void closeRightClaw() {
        rightClaw.setPosition(0.35);
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
        //claw rotator
        this.rotatorLevel = rotator;
        if(rotator == 0) clawRotator.setPosition(0.0);
        else if (rotator == 1) clawRotator.setPosition(0.35);
        else if (rotator == 2) clawRotator.setPosition(0.735);
        else if (rotator == 3) clawRotator.setPosition(1);

        //claw pivot
        if (pivot) clawPivot.setPosition(0.91);
        else clawPivot.setPosition(0.0);

        //set lift target
        setLiftPosition(position);

        //set lift power
        while (leftLift.isBusy() || rightLift.isBusy()) {
            for(DcMotor lift: lifts){
                if (lift.getCurrentPosition() > lift.getTargetPosition()) {
                    if ((lift.getCurrentPosition() < 900 || lift.getCurrentPosition() > 1300) && lift.getTargetPosition() != hang) {
                        //can depend on gravity
                        if (lift.getCurrentPosition() < 450) lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        else lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        lift.setPower(0.0);
                    } else lift.setPower(0.4);
                } else lift.setPower(0.8);
            }
        }
    }

    public void update(boolean rotateClaw) {
        //claw rotator
        if(rotateClaw){
            //ground: 0, low: 1, high: 2, all the way: 3
            if(rotatorLevel == 0) clawRotator.setPosition(0.5);
            else if (rotatorLevel == 1) clawRotator.setPosition(0.75);
            else if (rotatorLevel == 2) clawRotator.setPosition(1);
            else clawRotator.setPosition(1);
        }

        if (clawFlip) clawPivot.setPosition(0.29);
        else clawPivot.setPosition(0.95);

        if(armExtension.isBusy()) armExtension.setPower(0.6);
        else armExtension.setPower(0.0);

        //the actual lift part
        for (DcMotor lift : lifts) {
            if (lift.isBusy()) {
                if (lift.getCurrentPosition() > lift.getTargetPosition()) {
                    if (lift.getCurrentPosition() < 900 && lift.getTargetPosition() != hang) {
                        //can depend on gravity
                        if (lift.getCurrentPosition() < 450) lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        else lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        lift.setPower(0.0);
                    } else lift.setPower(0.4);
                } else lift.setPower(0.8);
            } else if (lift.getTargetPosition() == high || lift.getTargetPosition() == ground) {
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
    public double getArmExPosition() {return armExtension.getCurrentPosition();}
    public double getArmExTargetPosition() {return armExtension.getTargetPosition();}
}
