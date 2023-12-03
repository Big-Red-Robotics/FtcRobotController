package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;

import java.util.Arrays;
import java.util.List;

public class NewArm {
    public DcMotor leftLift, rightLift;
    public Servo clawRotator;
    public Servo leftClaw, rightClaw;
    List<DcMotor> lifts;

    public enum ArmState {intake, outtake, level1, hang, triggers, none}
    private ArmState currentState = ArmState.none;
    public boolean hang = false;
    public boolean outtake = false;
    public int intake = 0;

    public NewArm(HardwareMap hardwareMap){
        this.leftLift = hardwareMap.get(DcMotor.class, RobotConfig.liftL);
        this.rightLift = hardwareMap.get(DcMotor.class, RobotConfig.liftR);
        this.clawRotator = hardwareMap.get(Servo.class, RobotConfig.clawRotator);
        this.leftClaw = hardwareMap.get(Servo.class, RobotConfig.leftClaw);
        this.rightClaw = hardwareMap.get(Servo.class, RobotConfig.rightClaw);

        lifts = Arrays.asList(leftLift, rightLift);
        for(DcMotor lift: lifts){
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        rightLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setState (ArmState state) {
        currentState = state;
    }

    public void openLeftClaw() {
        leftClaw.setPosition(.33);
    }
    public void openRightClaw() {
        rightClaw.setPosition(0.95);
    }
    public void closeRightClaw() {
        rightClaw.setPosition(.5);
    }
    public void closeLeftClaw() {
        leftClaw.setPosition(.7);
    }
    public void openClaw() {
        leftClaw.setPosition(.33);
        rightClaw.setPosition(0.95);
    }
    public void closeClaw() {
        leftClaw.setPosition(0.7);
        rightClaw.setPosition(0.5);
    }

    public void setClawRotatorPosition(double position){
        //floor: 0.43, level1: 0.66, flipped: 1.00
        clawRotator.setPosition(position);
    }

    public void setLiftPower(double power) {
        outtake = false;
        hang = false;
        setState(ArmState.triggers);

        for (DcMotor lift: lifts) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setPower(power);
            lift.setTargetPosition(leftLift.getCurrentPosition());
        }
    }

    public void toPosition(ArmState armState){
        //claw rotator
        if(armState == ArmState.none) setClawRotatorPosition(clawRotator.getPosition());
        else if(armState == ArmState.intake) setClawRotatorPosition(0.43);
        else if (armState == ArmState.level1) setClawRotatorPosition(0.69);
        else setClawRotatorPosition(1);

        //set lift target
        for (DcMotor lift : lifts){
            switch (armState){
                case level1:
                    lift.setTargetPosition(225);
                    break;
                case hang:
                    lift.setTargetPosition(1100);
                    break;
                case outtake:
                    lift.setTargetPosition(1380);
                    break;
                default:
                    lift.setTargetPosition(0);
            }
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //set lift power
        while (leftLift.isBusy() || rightLift.isBusy()) {
            for(DcMotor lift: lifts){
                if (lift.getCurrentPosition() > lift.getTargetPosition()) {
                    if (lift.getCurrentPosition() < 900 && currentState != ArmState.hang) {
                        //can depend on gravity
                        if (lift.getCurrentPosition() < 400) lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        else lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        lift.setPower(0.0);
                    } else lift.setPower(0.4);
                } else lift.setPower(0.8);
            }
        }
    }

    public void update() {
        for (DcMotor lift : lifts) {
            //claw rotator
            if(intake == 0) clawRotator.setPosition(0.43);
            else if (intake == 1) clawRotator.setPosition(0.66);
            else clawRotator.setPosition(1);

            //the actual lift part
            if (currentState == ArmState.none) {
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (outtake && lift.getCurrentPosition() < 1360) {
                    setState(ArmState.outtake);
                }
            } else {
                if (currentState == ArmState.intake) {lift.setTargetPosition(0); outtake = false;}
                else if (currentState == ArmState.outtake) {lift.setTargetPosition(1380); hang = false; outtake = true;}
                else if (currentState == ArmState.hang) {lift.setTargetPosition(1100); hang = true; outtake = false;}
                else if (currentState == ArmState.level1) {lift.setTargetPosition(275); hang = false;}
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (lift.isBusy()) {
                    if (lift.getCurrentPosition() > lift.getTargetPosition()) {
                        if (lift.getCurrentPosition() < 900 && !hang) {
                            //can depend on gravity
                            if (lift.getCurrentPosition() < 400) lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            else lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                            lift.setPower(0.0);
                        } else lift.setPower(0.4);
                    } else lift.setPower(0.8);
                } else if (currentState == ArmState.outtake || currentState == ArmState.intake) {
                    setState(ArmState.none);
                    lift.setPower(0.0);
                }
            }
        }
    }

    public int getLiftPosition() {return (leftLift.getCurrentPosition() + rightLift.getCurrentPosition())/2;}
    public int getTargetPosition() {return (leftLift.getTargetPosition() + rightLift.getTargetPosition())/2;}
    public double getPower() {return (leftLift.getPower() + rightLift.getPower())/2;}
    public ArmState getCurrentState() {return currentState;}
}
