package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.functions.closeGrabber;
import static org.firstinspires.ftc.teamcode.functions.firstLift;
import static org.firstinspires.ftc.teamcode.functions.initServos;
import static org.firstinspires.ftc.teamcode.functions.move;
import static org.firstinspires.ftc.teamcode.functions.openGrabber;
import static org.firstinspires.ftc.teamcode.functions.openGrabberWide;
import static org.firstinspires.ftc.teamcode.functions.scaleInput;
import static org.firstinspires.ftc.teamcode.functions.secondLift;
import static org.firstinspires.ftc.teamcode.hardware.initHardwareMap;




/**
 * Created by dansm on 12/7/2017.
 */

@TeleOp(name="JDTeleOpJavaUsingRobot")

public class JDTeleopUsingRobot extends LinearOpMode{
    static public int firstLiftDirection = -1;
    static public int secondLiftDirection = -1;

    static public int getLiftDirection(int lift){
        if(lift == 1){
            return firstLiftDirection;
        }
        else if(lift == 2){
            return secondLiftDirection;
        }
        else{
            //Should not be called EVER if coded properly
            return 0;
        }
    }

    static public void setLiftDirection(int lift, int value){
        if(lift == 1){
            firstLiftDirection = value;
        }
        else if(lift == 2){
            secondLiftDirection = value;
        }
    }

   public void addTelemetry(String caption, String telemetryValue, boolean update){
        telemetry.addData(caption, telemetryValue);
        if(update){
            telemetry.update();
        }
    }

    @Override

    public void runOpMode() throws InterruptedException{
        //Code to run after init is pressed
        boolean hardwareMapState;

        hardwareMapState = initHardwareMap(hardwareMap);

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
                openGrabberWide();
            }

            firstLift(gamepad2);
            secondLift(gamepad2);
        }
    }
}
