package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.functions.*;
import static org.firstinspires.ftc.teamcode.hardware.*;



/**
 * Created by dansm on 12/7/2017.
 */

@TeleOp(name="JDTeleOpJavaArcade")

public class JDTeleopUsingArcade extends LinearOpMode{
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
            gamepad1LeftY = scaleInputFixedSpeed(gamepad1.left_stick_y);
            gamepad1LeftX = scaleInputFixedSpeed(gamepad1.left_stick_x);
            gamepad1RightX = scaleInputFixedSpeed(gamepad1.right_stick_x);


            if(gamepad1RightX != 0){
                if(gamepad1RightX<0){
                    //Turn Left

                    frontLeftDriveMotor.setPower(0);
                    frontRightDriveMotor.setPower(gamepad1RightX);
                    backLeftDriveMotor.setPower(0);
                    backRightDriveMotor.setPower(gamepad1RightX);
                }
                else if(gamepad1RightX>0){
                    //Turn Right

                    frontLeftDriveMotor.setPower(gamepad1RightX);
                    frontRightDriveMotor.setPower(0);
                    backLeftDriveMotor.setPower(gamepad1RightX);
                    backRightDriveMotor.setPower(0);
                }
            }
            else if(gamepad1LeftX==0){
                moveInAStraightLine(gamepad1LeftY);
            }
            else if(gamepad1LeftY > 0 && gamepad1LeftX < 0){
                //Forwards and Left
                double average = Math.abs((gamepad1LeftY+gamepad1LeftX)/2);

                frontLeftDriveMotor.setPower(-average);
                frontRightDriveMotor.setPower(0);
                backLeftDriveMotor.setPower(0);
                backRightDriveMotor.setPower(average);
            }
            else if(gamepad1LeftY > 0 && gamepad1LeftX > 0){
                //Forwards and Right
                double average = Math.abs((gamepad1LeftY+gamepad1LeftX)/2);

                frontLeftDriveMotor.setPower(0);
                frontRightDriveMotor.setPower(average);
                backLeftDriveMotor.setPower(-average);
                backRightDriveMotor.setPower(0);
            }
            else if(gamepad1LeftY < 0 && gamepad1LeftX < 0){
                //Backwards and Left
                double average = Math.abs((gamepad1LeftY+gamepad1LeftX)/2);

                frontLeftDriveMotor.setPower(0);
                frontRightDriveMotor.setPower(-average);
                backLeftDriveMotor.setPower(average);
                backRightDriveMotor.setPower(0);
            }
            else if(gamepad1LeftY < 0 && gamepad1LeftX > 0){
                //Backwards and Right
                double average = Math.abs((gamepad1LeftY+gamepad1LeftX)/2);

                frontLeftDriveMotor.setPower(average);
                frontRightDriveMotor.setPower(0);
                backLeftDriveMotor.setPower(0);
                backRightDriveMotor.setPower(-average);
            }




            if(gamepad2.a){
                closeGrabber(constants.BOTH_GRABBERS);
            }
            else if(gamepad2.b){
                openGrabber(constants.BOTH_GRABBERS);
            }
            else if(gamepad2.y){
                openGrabberWide(constants.BOTH_GRABBERS);
            }

            firstLift(gamepad2, this);
            secondLift(gamepad2, this );
        }
    }
}
