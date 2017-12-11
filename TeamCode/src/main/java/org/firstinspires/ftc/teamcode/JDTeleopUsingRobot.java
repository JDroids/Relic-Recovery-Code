package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import static org.firstinspires.ftc.teamcode.robot.*;

/**
 * Created by dansm on 12/7/2017.
 */

@TeleOp(name="JDTeleOpJavaUsingRobot")

public class JDTeleopUsingRobot extends LinearOpMode{
    int firstLiftDirection = -1;
    int secondLiftDirection = -1;

    @Override

    public void runOpMode() throws InterruptedException{
        //Code to run after init is pressed
        boolean hardwareMapState;
        //hardwareMapState = initHardwareMap(hardwareMap);
        if(!hardwareMapState){
            return;
        }

        double gamepad1LeftY;
        double gamepad1RightY;
        double gamepad1LeftX;
        double gamepad1RightX;

        waitForStart();
        //Code to run after play is pressed

        initServos();

        while(opModeIsActive()){
            gamepad1LeftY = scaleInput(gamepad1.left_stick_y);
            gamepad1RightY = scaleInput(gamepad1.right_stick_y);
            gamepad1LeftX = scaleInput(gamepad1.left_stick_x);
            gamepad1RightX = scaleInput(gamepad1.right_stick_x);

            move(gamepad1LeftY, gamepad1RightY, gamepad1LeftX, gamepad1RightX);

            if(gamepad2.a){
                closeGrabber();
            }
            else if(gamepad2.b){
                openGrabber();
            }
            else if(gamepad2.y){
                wideOpenGrabber();
            }

            firstLiftDirection = firstLift(firstLiftDirection);
            secondLiftDirection = secondLift(secondLiftDirection);
        }
    }
}