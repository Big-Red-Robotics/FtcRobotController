package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class PixelIndicator {

    public enum Colors{
        PURPLE,
        GREEN,
        YELLOW,
        WHITE,
    }

    public RevColorSensorV3 csL;
    public RevColorSensorV3 csR;
    public DcMotor light1;
    public PixelIndicator(HardwareMap hardwareMap){
        csL = hardwareMap.get(RevColorSensorV3.class,"CSL");
        csR = hardwareMap.get(RevColorSensorV3.class,"CSR");

        light1 = hardwareMap.get(DcMotor.class,"light");

    }

    public boolean isThereAnyPixel(){
        return isTherePixelL() || isTherePixelR();
    }
    public boolean isTherePixelL(){
        return getDistanceL() < 2.3;
    }

    public boolean isTherePixelR(){
        return getDistanceR() < 2.3;
    }
    public void turnOnLights(){
        light1.setPower(-0.5);
    }

    public void turnOffLights(){
        light1.setPower(0);
    }

    public double getDistanceL(){
        return csL.getDistance(DistanceUnit.CM);
    }

    public double getDistanceR(){
        return csR.getDistance(DistanceUnit.CM);
    }

    public boolean isGreen(){
        return false;
    }

    public boolean isPurple(){
        return false;
    }

    public boolean isWhite(){
        return false;
    }

    public boolean isYellow(){
        return false;
    }

}

