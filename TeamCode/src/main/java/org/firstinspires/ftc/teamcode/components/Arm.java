package org.firstinspires.ftc.teamcode.components;

import android.text.method.Touch;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;

import java.util.Arrays;
import java.util.List;

public class Arm {
    private final DcMotor leftLift, rightLift;
    private final Servo clawRotator;
    private final ServoImplEx clawPivot;
    private final Servo leftClaw, rightClaw;
    private final DcMotor armExtension;
    List<DcMotor> lifts;

    //arm position
    public static final int GROUND = 0,
                            AUTON = 350,
                            LOW = 450,
                            MIDDLE = 580,
                            HANG = 1100,
                            HIGH = 1380;

    private int rotatorLevel = 0;
    private boolean clawFlip = false;
    public boolean rightClawOpen = false, leftClawOpen = false;

    public boolean hang = false;

    private TouchSensor slideZeroReset;

    public Arm(HardwareMap hardwareMap){
        this.slideZeroReset = hardwareMap.get(TouchSensor.class,"touch");
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

    public void toPosition(int position, int rotator, boolean pivot, Telemetry t){
        //claw pivot
        if (pivot) clawPivot.setPosition(0.29);
        else clawPivot.setPosition(0.95);

        //set lift target
        setLiftPosition(position);
        leftLift.setPower(0.1);
        rightLift.setPower(0.1);

        if (armExtension.getTargetPosition() == 0){
            armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //the touch sensor is flipped
            if(slideZeroReset.isPressed()) {
                if (Math.abs(armExtension.getCurrentPosition()) < 50) armExtension.setPower(-0.1);
                armExtension.setPower(-0.6);
            }
            else armExtension.setPower(0.0);
        } else if(armExtension.isBusy()) {
            armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(getLiftTargetPosition() == LOW || getLiftTargetPosition() == MIDDLE) armExtension.setPower(0.6);
            else armExtension.setPower(1.0);
        } else armExtension.setPower(0.0);

        //set lift power
        while (leftLift.isBusy() || rightLift.isBusy()) {
            t.addData("target position", getLiftTargetPosition());
            t.addData("current position", getLiftPosition());
            t.update();
            for(DcMotor lift: lifts){
                if (lift.getCurrentPosition() > lift.getTargetPosition()) {
                    if ((lift.getCurrentPosition() < 700 || lift.getCurrentPosition() > 1300) && !hang) {
                        //can depend on gravity
                        if (lift.getCurrentPosition() < MIDDLE) lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        else lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        lift.setPower(0.0);
                    } else if(hang) lift.setPower(0.6);
                    else lift.setPower(0.3);
                } else if(lift.getCurrentPosition() > LOW && lift.getTargetPosition() == MIDDLE) lift.setPower(0.3);
                else lift.setPower(0.8);
            }
        }
        leftLift.setPower(0.5);
        rightLift.setPower(0.5);

        //claw rotator
        if(rotator == 0) clawRotator.setPosition(0.4);
        else if (rotator == 1) clawRotator.setPosition(0.5);
        else if (rotator == 2) clawRotator.setPosition(1);
    }

    public void update(boolean rotateClaw) {
        //this touch sensor is flipped
        if(!slideZeroReset.isPressed()) armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //claw rotator
        if(rotateClaw){
            //ground: 0, low: 1, middle: 5, all the way: 2
            if(rotatorLevel == 0) clawRotator.setPosition(0.4);
            else if (rotatorLevel == 1) clawRotator.setPosition(0.55);
            else if (rotatorLevel == 5) clawRotator.setPosition(0.5);
            else if (rotatorLevel == 2) clawRotator.setPosition(1);
            else if (rotatorLevel == 3) clawRotator.setPosition(0.8);
        }

        if (clawFlip) clawPivot.setPosition(0.29);
        else clawPivot.setPosition(0.95);

        if (armExtension.getTargetPosition() == 0){
            armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //the touch sensor is flipped
            if(slideZeroReset.isPressed()) {
                if (Math.abs(armExtension.getCurrentPosition()) < 50) armExtension.setPower(-0.1);
                armExtension.setPower(-0.6);
            }
            else armExtension.setPower(0.0);
        } else if(armExtension.isBusy()) {
            armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(getLiftTargetPosition() == LOW || getLiftTargetPosition() == MIDDLE) armExtension.setPower(0.6);
            else armExtension.setPower(1.0);
        } else armExtension.setPower(0.0);

        //the actual lift part
        for (DcMotor lift : lifts) {
            if (lift.isBusy()) {
                if(lift.getTargetPosition() == HIGH && lift.getCurrentPosition() > HANG + 50){
                    //can depend on gravity
                    lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    lift.setPower(0.0);
                } else if (lift.getCurrentPosition() > lift.getTargetPosition()) {
                    if (lift.getCurrentPosition() < 700 && !hang) {
                        //can depend on gravity
                        if ((lift.getCurrentPosition() < MIDDLE && lift.getTargetPosition() == 0) || (lift.getCurrentPosition() < MIDDLE + 100)) lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
