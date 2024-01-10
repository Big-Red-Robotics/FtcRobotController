package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;

import java.util.Arrays;
import java.util.List;

public class Arm {
    public DcMotor leftLift, rightLift;
    public Servo clawRotator;
    public Servo clawPivot;
    public Servo leftClaw, rightClaw;
    List<DcMotor> lifts;

    public enum ArmState {ground, low, middle, high, hang, manual, rest}
    private ArmState currentState = ArmState.rest;
    private int rotatorLevel = 0;
    private boolean clawFlip = false;

    public boolean hang = false, outtake = false;

    public Arm(HardwareMap hardwareMap){
        this.leftLift = hardwareMap.get(DcMotor.class, RobotConfig.liftL);
        this.rightLift = hardwareMap.get(DcMotor.class, RobotConfig.liftR);
        this.clawRotator = hardwareMap.get(Servo.class, RobotConfig.clawRotator);
        this.leftClaw = hardwareMap.get(Servo.class, RobotConfig.leftClaw);
        this.rightClaw = hardwareMap.get(Servo.class, RobotConfig.rightClaw);
        this.clawPivot = hardwareMap.get(Servo.class, RobotConfig.clawPivot);

        lifts = Arrays.asList(leftLift, rightLift);
        for(DcMotor lift: lifts){
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        rightLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setState (ArmState state, int rotatorLevel, boolean flipClaw) {
        currentState = state;
        this.rotatorLevel = rotatorLevel;
        this.clawFlip = flipClaw;
    }

    
    public void openLeftClaw() {
        leftClaw.setPosition(0.2);
    }
    public void openRightClaw() {
        rightClaw.setPosition(0.8);
    }
    public void closeRightClaw() {
        rightClaw.setPosition(0.5);
    }
    public void closeLeftClaw() {
        leftClaw.setPosition(0.5);
    }
    public void openClaw() {
        leftClaw.setPosition(0.2);
        rightClaw.setPosition(0.8);
    }
    public void closeClaw() {
        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0.5);
    }

    public void setClawRotatorPosition(double position){
        //floor: 0.001, level1: 0.4, flipped: .72, slight lift: 1
        clawRotator.setPosition(position);
    }

    public void setLiftPower(double power) {
        outtake = false;
        hang = false;
        setState(ArmState.manual, getRotatorLevel(), getClawFlip());

        for (DcMotor lift: lifts) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setPower(power);
            lift.setTargetPosition(leftLift.getCurrentPosition());
        }
    }

    public void toPosition(ArmState armState, int rotator, boolean pivot){
        //claw rotator
        //ground: 0, low: 1, high: 2, slight lift: 3
        if(rotator == 0) clawRotator.setPosition(0.007);
        else if (rotator == 1) clawRotator.setPosition(0.445);
        else if (rotator == 2) clawRotator.setPosition(0.77);
        else if (rotator == 3) clawRotator.setPosition(1);
        else clawRotator.setPosition(clawRotator.getPosition());

        if (pivot) clawPivot.setPosition(0.265);
        else clawPivot.setPosition(0.949);

        //set lift target
        for (DcMotor lift : lifts){
            switch (armState){
                case low:
                    lift.setTargetPosition(375);
                    break;
                case hang:
                    lift.setTargetPosition(1100);
                    break;
                case high:
                    lift.setTargetPosition(1380);
                    break;
                default:
                    lift.setTargetPosition(50);
            }
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //set lift power
        while (leftLift.isBusy() || rightLift.isBusy()) {
            for(DcMotor lift: lifts){
                if (lift.getCurrentPosition() > lift.getTargetPosition()) {
                    if (lift.getCurrentPosition() < 900 && currentState != ArmState.hang) {
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
        if(currentState != ArmState.manual){
            //claw rotator
            if(rotateClaw){
                //ground: 0, low: 1, high: 2, all the way: 3
                if(rotatorLevel == 0) clawRotator.setPosition(0.007);
                else if (rotatorLevel == 1) clawRotator.setPosition(0.445);
                else if (rotatorLevel == 2) clawRotator.setPosition(0.77);
                else clawRotator.setPosition(1);
            }

            if (clawFlip) clawPivot.setPosition(0.265);
            else clawPivot.setPosition(0.949);

            //the actual lift part
            for (DcMotor lift : lifts) {
                if (currentState == ArmState.rest) {
                    lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (outtake && lift.getCurrentPosition() < 1360) {
                        setState(ArmState.high, getRotatorLevel(), getClawFlip());
                    }
                } else {
                    if (currentState == ArmState.ground) {lift.setTargetPosition(0); outtake = false;}
                    else if (currentState == ArmState.high) {lift.setTargetPosition(1380); hang = false; outtake = true;}
                    else if (currentState == ArmState.hang) {lift.setTargetPosition(1100); hang = true; outtake = false;}
                    else if (currentState == ArmState.low) {lift.setTargetPosition(410); hang = false; outtake = false;}
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (lift.isBusy()) {
                        if (lift.getCurrentPosition() > lift.getTargetPosition()) {
                            if (lift.getCurrentPosition() < 900 && !hang) {
                                //can depend on gravity
                                if (lift.getCurrentPosition() < 450) lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                else lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                                lift.setPower(0.0);
                            } else lift.setPower(0.4);
                        } else lift.setPower(0.8);
                    } else if (currentState == ArmState.high || currentState == ArmState.ground) {
                        setState(ArmState.rest, getRotatorLevel(), getClawFlip());
                        lift.setPower(0.0);
                    }
                }
            }
        }
    }

    public int getLiftPosition() {return (leftLift.getCurrentPosition() + rightLift.getCurrentPosition())/2;}
    public int getTargetPosition() {return (leftLift.getTargetPosition() + rightLift.getTargetPosition())/2;}
    public double getPower() {return (leftLift.getPower() + rightLift.getPower())/2;}
    public ArmState getCurrentState() {return currentState;}
    public int getRotatorLevel() {return rotatorLevel;}
    public boolean getClawFlip() {return clawFlip;}
}
