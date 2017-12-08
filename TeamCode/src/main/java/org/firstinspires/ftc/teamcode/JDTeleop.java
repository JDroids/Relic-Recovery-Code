package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by dansm on 12/7/2017.
 */

@TeleOp(name="JDTeleop")

public class JDTeleop extends LinearOpMode{
    @Override

    DcMotor frontLeftDriveMotor = null;
    DcMotor frontRightDriveMotor = null;
    DcMotor backLeftDriveMotor = null;
    DcMotor backRightDriveMotor = null;

    public void runOpMode() throws InterruptedException{
        //Code to run after init is pressed

        //TODO: Get hardware map setup

        waitForStart();

        double gamepad1LeftY;
        double gamepad1RightY;
        double gamepad1LeftX;
        double gamepad1RightX;

        while(opModeIsActive()){
            gamepad1LeftY = scaleInput(gamepad1.left_stick_y);
            gamepad1RightY = scaleInput(gamepad1.right_stick_y);
            gamepad1LeftX = scaleInput(gamepad1.left_stick_x);
            gamepad1RightX = scaleInput(gamepad1.right_stick_x);

            move(gamepad1LeftY, gamepad1RightY, gamepad1LeftX, gamepad1RightX);

            //TODO: Get glyph grabber lift and closing/opening setup NOTE: SERVOS CAN'T MOVE DURING INIT THAT IS ILLEGAL
        }

        //Code to run after play is pressed
    }

    double STRAFING_LIMIT = 0.1;

    public void move(double leftY, double rightY, double leftX, double rightX){
        if(gamepadLeftX >= STRAFING_LIMIT && gamepadRightX >= STRAFING_LIMIT || gamepadLeftX <= STRAFING_LIMIT && gamepadRightX <= STRAFING_LIMIT){
            //To strafe either left or right
            frontLeftDriveMotor.setPower(leftX);
            frontRightDriveMotor.setPower(leftX);
            backLeftDriveMotor.setPower(-leftX);
            backRightDriveMotor.setPower(-leftX);
        }

        else{
            //To move forwards/backwards/turn with tank drive controls
            frontLeftDriveMotor.setPower(leftY);
            frontRightDriveMotor.setPower(leftY);
            backLeftDriveMotor.setPower(-rightY);
            backRightDriveMotor.setPower(-rightY);
        }
    }

    public double scaleInput(double dVal){
        double[] scaleArray = {0.0, 0.05, 0.09, 0.1, 0.12, 0.15, 0.18, 0.24, 0.3, 0.36, 0.43, 0.5, 0.6, 0.72, 0.85, 1.0, 1.0};

        int index;
        index = (int) (dVal * 16);
        index = java.lang.Math.abs(index);
        if(index > 16){
            index = 16;
        }

        double dScale;
        if(dVal < 0){
            dScale = -scaleArray[index]
        }
        else{
            dScale = scaleArray[index];
        }
    }
}
