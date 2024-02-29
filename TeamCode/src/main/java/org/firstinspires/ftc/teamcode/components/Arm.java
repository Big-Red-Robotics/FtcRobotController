package org.firstinspires.ftc.teamcode.components;

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
    private final ServoImplEx clawPivot, clawRotator;
    private final Servo leftClaw, rightClaw;
    private final DcMotor armExtension;
    public final TouchSensor slideZeroReset;
    List<DcMotor> lifts;

    //arm position
    public static final int GROUND = 0,
                            AUTON = 370,
                            VERY_LOW = 420,
                            LOW = 500,
                            MIDDLE = 600,
                            HANG = 1100,
                            HIGH = 1380;

    //claw pivot
    public static final double CP_FLIP = 0.945,
                               CP_HALF = 0.785,
                               CP_DEFAULT = 0.29;

    //claw rotator
    public static final double CR_STACK = 0.0,
                               CR_GROUND = 0.15,
                               CR_FRONT = 0.5,
                               CR_FLIP = 1.0;

    private int rotatorLevel = 0;
    private double clawPivotPosition = 0.0;
    public boolean rightClawOpen = false, leftClawOpen = false;

    public boolean hang = false;

    public Arm(HardwareMap hardwareMap){
        this.slideZeroReset = hardwareMap.get(TouchSensor.class,"touch");
        this.leftLift = hardwareMap.get(DcMotor.class, RobotConfig.liftL);
        this.rightLift = hardwareMap.get(DcMotor.class, RobotConfig.liftR);
        this.clawRotator = hardwareMap.get(ServoImplEx.class, RobotConfig.clawRotator);
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
        clawPivot.setPosition(CP_DEFAULT);

        clawRotator.setPwmRange(new PwmControl.PwmRange(1180, 1620));
        clawPivot.setPosition(CR_GROUND);
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

    public void setRotatorLevel(int level){
        rotatorLevel = level;
    }

    public void setClawPivotPosition(double flip){
        this.clawPivotPosition = flip;
    }

    //armEx
    public void setArmExtensionPosition(int position){
        armExtension.setTargetPosition(position);
    }

    //claw
    //TODO: this entire thing is flipped (right is left and left is right)
    public void openLeftClaw() {
        leftClaw.setPosition(1.0);
        leftClawOpen = true;
    }
    public void closeLeftClaw() {
        leftClaw.setPosition(0.3);
        leftClawOpen = false;
    }
    public void toggleLeftClaw(){
        if(leftClawOpen) closeLeftClaw();
        else openLeftClaw();
    }

    public void openRightClaw() {
        rightClaw.setPosition(0.0);
        rightClawOpen = true;
    }
    public void closeRightClaw() {
        rightClaw.setPosition(0.5);
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
    public void moveClawRotator(int level){
        //ground: 0, low: 1, middle: 5, all the way: 2
        if (level == 4) clawRotator.setPosition(CR_STACK);
        else if(level == 0) clawRotator.setPosition(CR_GROUND);
        else if (level == 1) clawRotator.setPosition(CR_FRONT);
        else if (level == 2) clawRotator.setPosition(CR_FLIP);
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
        if (pivot) clawPivot.setPosition(CP_FLIP);
        else clawPivot.setPosition(CP_DEFAULT);

//        clawPivot.setPosition(clawPivotPosition);

        //set lift target
        setLiftPosition(position);
        leftLift.setPower(0.1);
        rightLift.setPower(0.1);

        //if retrack back. Default retract at -0.6 unless
        if (armExtension.getTargetPosition() == 0){
            armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //the touch sensor is flipped
            while(slideZeroReset.isPressed()) {
                if (Math.abs(armExtension.getCurrentPosition()) < 50) armExtension.setPower(-0.1);
                else armExtension.setPower(-0.6);
            }
            armExtension.setPower(0.0);
        }
        while (armExtension.isBusy()) {
            // when it has a target.
            armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armExtension.setPower(0.6);
        }
        armExtension.setPower(0.0);

        //set lift power
        while (leftLift.isBusy() || rightLift.isBusy()) {
            t.addData("target position", getLiftTargetPosition());
            t.addData("current position", getLiftPosition());
            t.update();
            for(DcMotor lift: lifts){
                if (lift.getCurrentPosition() > lift.getTargetPosition()) {
                    if(hang) lift.setPower(1.0);
                    else if (lift.getCurrentPosition() < 700 || lift.getCurrentPosition() > 1300) {
                        //can depend on gravity
                        if (lift.getCurrentPosition() < MIDDLE) lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        else lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        lift.setPower(0.0);
                    }
                    else lift.setPower(0.3);
                } else if(lift.getCurrentPosition() > LOW && lift.getTargetPosition() == MIDDLE) lift.setPower(0.3);
                else lift.setPower(0.8);
            }

            if(getLiftTargetPosition() == 0 && getLiftPosition() < 15) break;
        }
        leftLift.setPower(0.5);
        rightLift.setPower(0.5);

        //claw rotator
        moveClawRotator(rotator);
    }

    public void update(boolean rotateClaw) {
        //this touch sensor is flipped
        if(!slideZeroReset.isPressed()) armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //claw rotator
        if(rotateClaw) moveClawRotator(rotatorLevel);

        clawPivot.setPosition(clawPivotPosition);

        if (armExtension.getTargetPosition() == 0){
            armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //the touch sensor is flipped
            if(slideZeroReset.isPressed()) {
                if (Math.abs(armExtension.getCurrentPosition()) < 50) armExtension.setPower(-0.1);
                else armExtension.setPower(-0.6);
            }
            else armExtension.setPower(0.0);
        } else if(armExtension.isBusy()) {
            armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(armExtension.getTargetPosition() == 100) armExtension.setPower(1.0);
            else if(getLiftTargetPosition() == LOW || getLiftTargetPosition() == MIDDLE || getLiftTargetPosition() == VERY_LOW || getLiftTargetPosition() == AUTON)
                armExtension.setPower(0.6);
            else armExtension.setPower(1.0);
        } else armExtension.setPower(0.0);

        //the actual lift part
        for (DcMotor lift : lifts) {
            if (lift.isBusy()) {
                if(hang) lift.setPower(1.0);
                else if(lift.getTargetPosition() == HIGH && lift.getCurrentPosition() > HANG + 20){
                    //can depend on gravity
                    lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    lift.setPower(0.0);
                } else if (lift.getCurrentPosition() > lift.getTargetPosition()) {
                    if (lift.getCurrentPosition() < 700) {
                        //can depend on gravity
                        if ((lift.getCurrentPosition() < MIDDLE && lift.getTargetPosition() == 0) || (lift.getCurrentPosition() < MIDDLE + 100)) lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        else lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        lift.setPower(0.0);
                    } else lift.setPower(0.7);
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

    public double getArmExPower() {return armExtension.getPower();}
    public int getArmExPosition() {return armExtension.getCurrentPosition();}
    public int getArmExTargetPosition() {return armExtension.getTargetPosition();}
}
