package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.NewArm;
import org.firstinspires.ftc.teamcode.components.NewVision;

import org.firstinspires.ftc.teamcode.utility.RobotConfig;
import org.firstinspires.ftc.teamcode.utility.teaminfo.InitialSide;
import org.firstinspires.ftc.teamcode.utility.teaminfo.TeamColor;

@Autonomous(name="Oct 29 Autonomous")
public class FirstAutonomous extends LinearOpMode {
    /*
    see
    https://github.com/Big-Red-Robotics/PowerPlay/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/autonomous/Basic_Autonomous.java
    for last season's autonomous using built-in encoders.
     */

    //indicator
    enum Indicator {RIGHT, MIDDLE, LEFT};
    Indicator indicator;

    boolean isRight, isRed;

    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap);
        NewArm arm = new NewArm(hardwareMap);
        NewVision vision = new NewVision(hardwareMap);

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
            telemetry.addData("Team color", RobotConfig.teamColor);
            telemetry.addData("Initial side", RobotConfig.initialSide);

            isRight = RobotConfig.initialSide == InitialSide.RIGHT;
            isRed = RobotConfig.teamColor == TeamColor.RED;

            /*
            read indicator value with vision.getIndicator()
            left half of the screen: 1
            right half of the screen: 2
            cannot find indicator: 3
             */
            telemetry.addData("isRight", isRight);
            int rawIndicatorValue = vision.getIndicator();
            if(isRight != isRed){
                if(rawIndicatorValue == 1) indicator = Indicator.MIDDLE;
                else if(rawIndicatorValue == 2) indicator = Indicator.RIGHT;
                else if(rawIndicatorValue == 3) indicator = Indicator.LEFT;
            } else {
                if(rawIndicatorValue == 1) indicator = Indicator.LEFT;
                else if(rawIndicatorValue == 2) indicator = Indicator.MIDDLE;
                else if(rawIndicatorValue == 3) indicator = Indicator.RIGHT;
            }

            telemetry.addData("Detected indicator", indicator);
            telemetry.addData("indicator area", vision.indicatorProcessor.pixel.area());
            telemetry.addData("indicator x", vision.indicatorProcessor.pixel.x);
            telemetry.update();

            sleep(100);
        }

        //after start button pressed
        arm.closeClaw();

        //place indicator
        chassis.runToPosition(-1000, -1600, -1600, -1000);
        chassis.resetEncoders();
        if(indicator == Indicator.LEFT) chassis.runToPosition(800, -800, 800, -800);
        else if(indicator == Indicator.RIGHT) chassis.runToPosition(-800, 800, -800, 800);
        chassis.resetEncoders();
        chassis.runToPosition(-100, -100, -100, -100);
        arm.openClaw();
        sleep(20);
        chassis.runToPosition(0,0,0,0);
        if(indicator == Indicator.RIGHT) chassis.runToPosition(800, -800, 800, -800);
        else if(indicator == Indicator.LEFT) chassis.runToPosition(-800, 800, -800, 800);
        chassis.resetEncoders();
        chassis.runToPosition(1000,1000,1000,1000);
        chassis.resetEncoders();
        //TODO: correspond for multiple initial location
        if(isRight && isRed) chassis.runToPosition(1700, -1700, -1700, 1700);
        else if (!isRight && !isRed) chassis.runToPosition(-1700, 1700, 1700, -1700);
    }
}
