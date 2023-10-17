package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;

import java.util.Arrays;
import java.util.List;

public class NewArm {
    //look at teamcode/components/old/Arm.java for last season's code.

    public DcMotor leftLift, rightLift;
//    public DcMotor clawRotator;
    public Servo clawR, clawL;
    List<DcMotor> lifts;

    //TODO: configure this, idk if we will need it for Oct 29.
    public final int intakeArmPosition = 0;
    public final int outtakeArmPosition = 0;
    public int armTarget;

    //TODO: enum for state (probably intake, outtake, & none?)
    public enum ArmState {intake, outtake, hang, none};

    public ArmState currentState = ArmState.none;

    public NewArm(HardwareMap hardwareMap){
        this.leftLift = hardwareMap.get(DcMotor.class, RobotConfig.liftL);
        this.rightLift = hardwareMap.get(DcMotor.class, RobotConfig.liftR);
//        this.clawRotator = hardwareMap.get(DcMotor.class, RobotConfig.clawRotator);
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
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    public void openClaw(){
        clawR.setPosition(0.0);
        clawL.setPosition(0.0);
    }

    public void closeClaw(){
        clawR.setPosition(0.5);
        clawL.setPosition(0.5);
    }

    public void update(Telemetry t) {
        if(currentState == ArmState.none){
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            if(currentState == ArmState.intake) {
                rightLift.setTargetPosition(50);
                leftLift.setTargetPosition(50);
            } else if (currentState == ArmState.outtake){
                rightLift.setTargetPosition(1350);
                leftLift.setTargetPosition(1350);
            } else {
                rightLift.setTargetPosition(1100);
                leftLift.setTargetPosition(1100);
            }

            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(rightLift.isBusy() && leftLift.isBusy()){
                rightLift.setPower(0.5);
                leftLift.setPower(0.5);
                t.addData("current pos", getLiftPosition());
                t.update();
            } else {
                rightLift.setPower(0.0);
                leftLift.setPower(0.0);
                setState(ArmState.none);
            }
        }
    }

    //TODO: maybe useful getters such as...
    public double getClawPosition(){return (clawL.getPosition() + clawR.getPosition())/2;}
    public int getLiftPosition(){return (leftLift.getCurrentPosition() + rightLift.getCurrentPosition())/2;}
    public int getTargetPosition(){return (leftLift.getTargetPosition() + rightLift.getTargetPosition())/2;}
    public double getPower(){return (leftLift.getPower() + rightLift.getPower())/2;}
}
