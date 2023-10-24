package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;

import java.util.Arrays;
import java.util.List;

public class NewArm {
    public DcMotor leftLift, rightLift;
    public Servo clawRotator;
    public Servo clawR, clawL;
    List<DcMotor> lifts;

    public enum ArmState {intake, outtake, hang, none};
    public ArmState currentState = ArmState.none;
    public boolean hang = false;

    public NewArm(HardwareMap hardwareMap){
        this.leftLift = hardwareMap.get(DcMotor.class, RobotConfig.liftL);
        this.rightLift = hardwareMap.get(DcMotor.class, RobotConfig.liftR);
        this.clawRotator = hardwareMap.get(Servo.class, RobotConfig.clawRotator);
        this.clawR = hardwareMap.get(Servo.class, RobotConfig.clawR);
        this.clawL = hardwareMap.get(Servo.class, RobotConfig.clawL);

        clawR.setDirection(Servo.Direction.REVERSE);

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

    public void setLiftPower(double power, Telemetry t) {
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
        clawR.setPosition(0.0);
        clawL.setPosition(0.0);
    }

    public void closeClaw(){
        clawR.setPosition(0.5);
        clawL.setPosition(0.5);
    }

    public void update() {
        for (DcMotor lift : lifts) {
            //claw stopper
            if(lift.getTargetPosition() == 0 && lift.getCurrentPosition() < 20) clawRotator.setPosition(0.0);
            else clawRotator.setPosition(0.5);

            //the actual lift part
            if (currentState == ArmState.none) lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            else {
                if (currentState == ArmState.intake) lift.setTargetPosition(0);
                else if (currentState == ArmState.outtake) {lift.setTargetPosition(1370); hang = false;}
                else {lift.setTargetPosition(1100); hang = true;}//currentState = ArmState.hang
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                /*if (lift.isBusy()) {
                    if (lift.getCurrentPosition() > lift.getTargetPosition()) {
                        if (lift.getCurrentPosition() < 350) {
                            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            lift.setPower(0.0);
                        } else if (lift.getCurrentPosition() < 500) {
                            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                            lift.setPower(0.0);
                        } else {
                            lift.setPower(0.5);
                        }
                    } else if (lift.getCurrentPosition() < lift.getTargetPosition()) {
                        lift.setPower(0.5);
                    }
                } else {
                    lift.setPower(0.0);
                    setState(ArmState.none);
                }*/

                if (lift.isBusy()) {
                    if (lift.getCurrentPosition() > lift.getTargetPosition()) {
                        if (lift.getCurrentPosition() < 800 && !hang) {
                            //can depend on gravity
                            if (lift.getCurrentPosition() < 250) lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            else lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                            lift.setPower(0.0);
                        } else lift.setPower (0.4);
                    } else lift.setPower(0.8);
                } else if (!lift.isBusy() && currentState == ArmState.outtake) {
                    if (lift.getCurrentPosition() > 1350) lift.setPower(0.0);
                    else setState(ArmState.outtake);
                } else if (!lift.isBusy() && currentState == ArmState.intake) {
                    lift.setPower(0.0);
                    setState(ArmState.none);
                }
            }
        }
    }

    public double getClawPosition() {return (clawL.getPosition() + clawR.getPosition())/2;}
    public int getLiftPosition() {return (leftLift.getCurrentPosition() + rightLift.getCurrentPosition())/2;}
    public int getTargetPosition() {return (leftLift.getTargetPosition() + rightLift.getTargetPosition())/2;}
    public double getPower() {return (leftLift.getPower() + rightLift.getPower())/2;}
}
