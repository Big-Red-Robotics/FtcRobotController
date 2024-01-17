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
    public Servo claw;
    List<DcMotor> lifts;

    public enum ArmState {intake, outtake, hang, none};
    public ArmState currentState = ArmState.none;
    public boolean hang = false;
    public boolean outtake = false;

    public Arm(HardwareMap hardwareMap){
        this.leftLift = hardwareMap.get(DcMotor.class, RobotConfig.liftL);
        this.rightLift = hardwareMap.get(DcMotor.class, RobotConfig.liftR);
        this.clawRotator = hardwareMap.get(Servo.class, RobotConfig.clawRotator);
        this.claw = hardwareMap.get(Servo.class, RobotConfig.claw);

        lifts = Arrays.asList(leftLift, rightLift);
        for(DcMotor lift: lifts){
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        rightLift.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setState (ArmState state){
        currentState = state;
    }

    public void setLiftPower(double power) {
        outtake = false;
        setState(ArmState.none);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLift.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.FORWARD);

        leftLift.setPower(power);
        rightLift.setPower(power);

        leftLift.setTargetPosition(leftLift.getCurrentPosition());
        rightLift.setTargetPosition(rightLift.getCurrentPosition());
    }

    public void openClaw(){
        claw.setPosition(0.6);
    }

    public void closeClaw(){
        claw.setPosition(0.32);
    }

    public void update() {
        for (DcMotor lift : lifts) {
            //claw stopper
            if(lift.getTargetPosition() == 0 && lift.getCurrentPosition() < 100) clawRotator.setPosition(0.0);
            else clawRotator.setPosition(0.5);

            //the actual lift part
            if (currentState == ArmState.none) {
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (outtake && lift.getCurrentPosition() < 1360 && lift.getCurrentPosition() > 300) {
                    setState(ArmState.outtake);
                }
            } else {
                if (currentState == ArmState.intake) {lift.setTargetPosition(0); outtake = false;}
                else if (currentState == ArmState.outtake) {lift.setTargetPosition(1380); hang = false; outtake = true;}
                else {lift.setTargetPosition(1100); hang = true; outtake = false;}
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (lift.isBusy()) {
                    if (lift.getCurrentPosition() > lift.getTargetPosition()) {
                        if (lift.getCurrentPosition() < 800 && !hang) {
                            //can depend on gravity
                            if (lift.getCurrentPosition() < 250) lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            else lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                            lift.setPower(0.0);
                        } else lift.setPower(0.4);
                    } else lift.setPower(0.8);
                } else if (!hang) {
                    setState(ArmState.none);
                    lift.setPower(0.0);
                }
            }
        }
    }

    public int getLiftPosition() {return (leftLift.getCurrentPosition() + rightLift.getCurrentPosition())/2;}
    public int getTargetPosition() {return (leftLift.getTargetPosition() + rightLift.getTargetPosition())/2;}
    public double getPower() {return (leftLift.getPower() + rightLift.getPower())/2;}
}
