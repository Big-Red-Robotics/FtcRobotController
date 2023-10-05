package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;

import java.util.Arrays;
import java.util.List;

public class NewArm {
    //look at teamcode/components/old/Arm.java for last season's code.

    private List<DcMotor> arms;
    public DcMotor leftArm, rightArm;
    public Servo clawRotator;
    public Servo claw;

    //TODO: configure this, idk if we will need it for Oct 29.
    public final int intakeArmPosition = 0;
    public final int outtakeArmPosition = 0;
    public int armTarget;

    //TODO: enum for state (probably intake, outtake, & none?)
    public enum ArmState {drop, raise, none};

    public ArmState currentState = ArmState.none;

    public NewArm(HardwareMap hardwareMap){
        //TODO: Configure teamcode/utility/RobotConfig with the right name
        this.leftArm = hardwareMap.get(DcMotor.class, RobotConfig.lifts[0]);
        this.rightArm = hardwareMap.get(DcMotor.class, RobotConfig.lifts[1]);
        this.clawRotator = hardwareMap.get(Servo.class, RobotConfig.clawRotator);
        this.claw = hardwareMap.get(Servo.class, RobotConfig.claw);

        rightArm.setDirection(DcMotor.Direction.REVERSE);

        arms = Arrays.asList(leftArm, rightArm);
        for(DcMotor arm: arms){
            //TODO: idk if this applies to this season, adjust as preferred
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    //TODO: make useful methods such as...
    public void openClaw() {}
    public void closeClaw() {}
    public void setArmPower(Gamepad gamepad, double power) {}

    public boolean isBusy(){
        return leftArm.isBusy() || rightArm.isBusy();
    }
    public void runToPosition(int position) {}

    //TODO: maybe useful getters such as...
    public int getCurrentPosition(){return 0;}
    public int getTargetPosition(){return 0;}
    public double getPower(){return 0.0;}

    //or maybe we don't even need to make things this complicated for Oct 29.
    //essentially if we can intake & outtake in one line, that would be good.
    //claw presets, rotator presets & arm presets?
    //probably use enum so that we can monitor current state easily from other classes.
}
