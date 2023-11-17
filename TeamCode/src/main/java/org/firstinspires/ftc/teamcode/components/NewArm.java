package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;

import java.util.Arrays;
import java.util.List;

public class NewArm {
    public DcMotor leftLift, rightLift;
    public Servo clawRotator;
    public Servo leftClaw, rightClaw;
    List<DcMotor> lifts;

    public enum ArmState {intake, outtake, hang, triggers, none};
    public ArmState currentState = ArmState.none;
    public boolean hang = false;
    public boolean outtake = false;
    public boolean intake = true;

    ElapsedTime runtime = new ElapsedTime();

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

        rightLift.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setState (ArmState state){
        currentState = state;
    }

    public void setLiftPower(double power, Telemetry t) {
        outtake = false;
        intake = false;
        setState(ArmState.triggers);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLift.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.FORWARD);

        leftLift.setPower(power);
        rightLift.setPower(power);

        leftLift.setTargetPosition(leftLift.getCurrentPosition());
        rightLift.setTargetPosition(rightLift.getCurrentPosition());
    }

    public void flipWrist() {
        intake = false;
    }

    public void closeWrist() {
        intake = true;
    }

    public void openClaw() {
        leftClaw.setPosition(.33);
        rightClaw.setPosition(0.95);
    }

    public void openLeftClaw() {
        leftClaw.setPosition(.33);
    }

    public void openRightClaw() {
        rightClaw.setPosition(0.95);
    }

    public void closeClaw() {
        leftClaw.setPosition(0.7);
        rightClaw.setPosition(0.5);
    }

    public void update() {
        for (DcMotor lift : lifts) {
            //claw stopper
            if(intake) clawRotator.setPosition(0.44);
            else clawRotator.setPosition(1);

            //the actual lift part
            if (currentState == ArmState.none) {
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (outtake && lift.getCurrentPosition() < 1360 && lift.getCurrentPosition() > 300) {
                    setState(ArmState.outtake);
                }
            } else {
                if (currentState == ArmState.intake) {lift.setTargetPosition(0); outtake = false; intake = true;}
                else if (currentState == ArmState.outtake) {lift.setTargetPosition(1380); hang = false; outtake = true; intake = false;}
                else if (currentState == ArmState.hang) {lift.setTargetPosition(1100); hang = true; outtake = false; intake = false; }
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
                } else if (!hang && currentState != ArmState.triggers) {
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
