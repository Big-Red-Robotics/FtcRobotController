package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;

import java.util.Arrays;
import java.util.List;

public class NewIntake {
    public DcMotor intake;
    List<DcMotor> lifts;



    public NewIntake(HardwareMap hardwareMap){
        this.intake = hardwareMap.get(DcMotor.class, RobotConfig.intakeName);

        this.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void run(){
        this.intake.setPower(0.7);
    }

    public void reverse(){this.intake.setPower(-0.7);}

    public void stop(){
        this.intake.setPower(0);
    }


}
