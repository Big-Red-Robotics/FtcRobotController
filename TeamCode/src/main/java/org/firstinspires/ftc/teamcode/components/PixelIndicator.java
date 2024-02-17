package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PixelIndicator {

    public enum Colors{
        PURPLE,
        GREEN,
        YELLOW,
        WHITE,
        NONE
    }

    public RevColorSensorV3 colorSensorLeft, colorSensorRight;

    public PixelIndicator(HardwareMap hardwareMap){
        colorSensorLeft = hardwareMap.get(RevColorSensorV3.class,"CSL");
        colorSensorRight = hardwareMap.get(RevColorSensorV3.class,"CSR");
    }

    public Colors pixelColorRight(){
        int green = colorSensorRight.green();
        int red = colorSensorRight.red();
        int blue = colorSensorRight.blue();
        int alpha = colorSensorRight.alpha();

        if(isTherePixelR()){
            if(green > 585 && blue > 245 && red > 200 && (alpha > 340 && alpha < 600)) return Colors.GREEN;
            else if((green > 600 && green < 1000) && blue > 950 && red > 530 && alpha > 750) return Colors.PURPLE;
            else if(green > 1700 && blue > 1450 && red > 1000 && alpha > 1400) return Colors.WHITE;
            else if(green > 900 && blue > 250 && red > 650 && (alpha > 615 && alpha < 800)) return Colors.YELLOW;
        }
        return Colors.NONE;
    }

    public boolean isTherePixelL(){
        return getDistanceL() < 2.3;
    }
    public boolean isTherePixelR(){
        return getDistanceR() < 2.3;
    }
    public boolean isThereAnyPixel(){
        return isTherePixelL() || isTherePixelR();
    }

    public double getDistanceL(){
        return colorSensorLeft.getDistance(DistanceUnit.CM);
    }
    public double getDistanceR(){
        return colorSensorRight.getDistance(DistanceUnit.CM);
    }
}

