package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.RobotConfig;

import org.arcrobotics.ftclib.controller.PIDController;

import java.util.Arrays;
import java.util.List;

@Config
@TeleOp
public class ArmPIDF extends OpMode{
    public final DcMotorEx leftLift, rightLift;
    List<DcMotorEx> lifts;

    //PIDF CONTROL
    private final PIDController pidController;
    public static double p = 0, i = 0, d = 0, f = 0;
    private static int target = 0;

    private final double ticksInDegree = 700 / 180.0;


    @Override
    public void init(){
        this.leftLift = hardwareMap.get(DcMotorEx.class, RobotConfig.liftL);
        this.rightLift = hardwareMap.get(DcMotorEx.class, RobotConfig.liftR);

        pidController = new PIDController(p, i, d, f);
    }

    @Override
    public void loop(){
        pidController.setPID(p, i, d);
        int leftPos = leftLift.getCurrentPosition();
        double pid = pidController.calculate(leftLiftPos, target);
        double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;

        double power = pid + ff;

        leftLift.setPower(power);
    }
}
