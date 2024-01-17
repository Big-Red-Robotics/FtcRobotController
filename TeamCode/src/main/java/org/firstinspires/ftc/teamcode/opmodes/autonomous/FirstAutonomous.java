package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Vision;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.teaminfo.InitialSide;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;

@Autonomous(name="Oct 29 Autonomous")
public class FirstAutonomous extends LinearOpMode {
    //indicator
    enum Indicator {RIGHT, MIDDLE, LEFT};
    Indicator indicator;

    //team info
    boolean isRight, isRed;

    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Vision vision = new Vision(hardwareMap);

        //open claw for 18-inch restriction
        arm.openClaw();

        telemetry.addLine("waiting to start!");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            //set team side
            if(gamepad1.b || gamepad2.b) RobotConfig.teamColor = TeamColor.RED;
            if(gamepad1.x || gamepad2.x) RobotConfig.teamColor = TeamColor.BLUE;
            if(gamepad1.dpad_right || gamepad2.dpad_right) RobotConfig.initialSide = InitialSide.RIGHT;
            if(gamepad1.dpad_left || gamepad2.dpad_left) RobotConfig.initialSide = InitialSide.LEFT;
            isRight = RobotConfig.initialSide == InitialSide.RIGHT;
            isRed = RobotConfig.teamColor == TeamColor.RED;
            telemetry.addData("Team color", RobotConfig.teamColor);
            telemetry.addData("Initial side", RobotConfig.initialSide);

            /*
            read indicator value with vision.getIndicator()
            left half of the screen: 1
            right half of the screen: 2
            cannot find indicator: 3
             */
            int rawIndicatorValue = vision.getIndicator();
            if(!isRight){
                if(rawIndicatorValue == 1) indicator = Indicator.LEFT;
                else if(rawIndicatorValue == 2) indicator = Indicator.MIDDLE;
                else if(rawIndicatorValue == 3) indicator = Indicator.RIGHT;
            } else {
                if(rawIndicatorValue == 1) indicator = Indicator.MIDDLE;
                else if(rawIndicatorValue == 2) indicator = Indicator.RIGHT;
                else if(rawIndicatorValue == 3) indicator = Indicator.LEFT;
            }

            telemetry.addData("Detected indicator", indicator);
            telemetry.update();

            sleep(100);
        }

        //after start button pressed
        arm.closeClaw();

        //place indicator
        if(isRight)chassis.runToPosition(-1500, -1000, -1000, -1500);
        else chassis.runToPosition(-1000, -1500, -1500, -1000);
        chassis.resetEncoders();
        if(indicator == Indicator.RIGHT) chassis.runToPosition(800, -700, 800, -700);
        else if(indicator == Indicator.LEFT) chassis.runToPosition(-700, 800, -700, 800);
        chassis.resetEncoders();
        arm.openClaw();
        sleep(20);
        chassis.runToPosition(150,150,150,150);
        if(indicator == Indicator.LEFT) chassis.runToPosition(800, -700, 800, -700);
        else if(indicator == Indicator.RIGHT) chassis.runToPosition(-700, 800, -700, 800);
        chassis.resetEncoders();
        if(isRight == isRed){
            if(isRight) chassis.runToPosition(200, -200, -200, 200);
            else chassis.runToPosition(-200, 200, 200, -200);
            chassis.resetEncoders();
            chassis.runToPosition(700, 700, 700, 700);
            chassis.resetEncoders();
            if(isRight) chassis.runToPosition(1600, -1600, -1600, 1600);
            else chassis.runToPosition(-1600, 1600, 1600, -1600);
        }
    }
}
