package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.constants.*;
import static org.firstinspires.ftc.teamcode.hardware.*;
import static org.firstinspires.ftc.teamcode.JDTeleopUsingRobot.*;

/**
 * Created by dansm on 12/13/2017.
 */


public class functions{
    static public double scaleInput(double dVal) throws InterruptedException {
        double result = Math.pow(dVal, 3);
        if (result > 0.7) {
            result = 0.7;
        }
        else if (result < -0.7) {
            result = -0.7;
        }
        return result;
    }

    static public void setGrabber(double leftServoPosition, double rightServoPosition) throws InterruptedException{
        servoGrabberLeft.setPosition(leftServoPosition);
        servoGrabberRight.setPosition(rightServoPosition);
    }

    static public void setJewelPosition(double jewelKnockerPosition, double jewelArmPosition){
        jewelKnocker.setPosition(jewelKnockerPosition);
        jewelArm.setPosition(jewelArmPosition);
    }

    static public void initServos() throws InterruptedException{
        setGrabber(SERVO_GRABBER_INIT_POSITION[0], SERVO_GRABBER_INIT_POSITION[1]);
        setJewelPosition(JEWEL_KNOCKER_INIT_POSITION, JEWEL_ARM_INIT_POSITION);
    }

    static public void closeGrabber() throws InterruptedException{
        setGrabber(SERVO_GRABBER_CLOSE_POSITION[0], SERVO_GRABBER_CLOSE_POSITION[1]);
    }

    static public void openGrabber() throws InterruptedException{
        setGrabber(SERVO_GRABBER_OPEN_POSITION[0], SERVO_GRABBER_OPEN_POSITION[1]);
    }

    static public void openGrabberWide() throws InterruptedException{
        setGrabber(SERVO_GRABBER_WIDE_OPEN_POSITION[0], SERVO_GRABBER_WIDE_OPEN_POSITION[1]);
    }

    static public void move(double leftY, double rightY, double leftX, double rightX) throws InterruptedException{
        if(leftX >= STRAFING_LIMIT && rightX >= STRAFING_LIMIT || leftX <= -STRAFING_LIMIT && rightX <= -STRAFING_LIMIT){
            //To strafe either left or right
            frontLeftDriveMotor.setPower(-leftX);
            frontRightDriveMotor.setPower(-leftX);
            backLeftDriveMotor.setPower(leftX);
            backRightDriveMotor.setPower(leftX);
        }

        else{
            //To move forwards/backwards/turn with tank drive controls
            frontLeftDriveMotor.setPower(leftY);
            frontRightDriveMotor.setPower(-rightY);
            backLeftDriveMotor.setPower(leftY);
            backRightDriveMotor.setPower(-rightY);
        }

    }

    static public void firstLift(Gamepad gamepad2) throws InterruptedException{
        JDTeleopUsingRobot runningOpMode = new JDTeleopUsingRobot();
        
        if(!firstLiftSwitch.getState() && getLiftDirection(1) == 1){
            runningOpMode.addTelemetry("First Lift", "Top Limit Reached - Move Down", true);

            if (gamepad2.left_stick_y > 0) {
                //Move down at a slow speed as gravity is pulling it down
                firstGlyphLift.setPower(0.2);

                //Sleep allows sensor to move away from the magnet
                Thread.sleep(200);
            }
            else {
                firstGlyphLift.setPower(0);
            }
        }
        else if(!firstLiftSwitch.getState() && getLiftDirection(1) == -1){
            runningOpMode.addTelemetry("First Lift", "Bottom Limit Reached - Move Up", true);

            if(gamepad2.left_stick_y < 0){
                secondGlyphLift.setPower(-0.5);

                Thread.sleep(400);

                setLiftDirection(1, 1);
            }
            else{
                secondGlyphLift.setPower(0);
            }
        }
        else{
            if(gamepad2.left_stick_y > 0){
                //Move slow regardless of input as gravity is pulling down
                firstGlyphLift.setPower(0.2);
            }
            else{
                firstGlyphLift.setPower(scaleInput(gamepad2.left_stick_y));
            }
            if(gamepad2.left_stick_y < 0){
                setLiftDirection(1, 1);
            }
            else if(gamepad2.left_stick_y > 0){
                setLiftDirection(1, 1);
            }

            runningOpMode.addTelemetry("First Lift", "Can move freely", true);
        }
    }

    static public void secondLift(Gamepad gamepad2) throws InterruptedException{
        JDTeleopUsingRobot runningOpMode = new JDTeleopUsingRobot();

        if(!secondLiftSwitch.getState() && getLiftDirection(2) == 1) {
            runningOpMode.addTelemetry("Second Lift", "Top Limit Reached - Move Down", true);

            if (gamepad2.right_stick_y > 0) {
                //Move down at a slow speed as gravity is pulling it down
                secondGlyphLift.setPower(-0.3);

                Thread.sleep(500);

                setLiftDirection(2, -1);
            } else {
                secondGlyphLift.setPower(0);
            }
        }
        else if(!secondLiftSwitch.getState() && getLiftDirection(2) == -1){
            runningOpMode.addTelemetry("Second Lift", "Bottom Limit Reached - Move Down", true);

            if(gamepad2.right_stick_y < 0){
                secondGlyphLift.setPower(0.5);

                Thread.sleep(500);

                setLiftDirection(1, 1);
            }
            else{
                secondGlyphLift.setPower(0);
            }
        }
        else{
            if(gamepad2.right_stick_y > 0){
                //Move slow regardless of input as gravity is pulling down
                secondGlyphLift.setPower(0.2);
            }
            else{
                secondGlyphLift.setPower(gamepad2.right_stick_y/2);
            }
            if(gamepad2.right_stick_y < 0){
                getLiftDirection(1);
            }
            else if(gamepad2.right_stick_y > 0){
                getLiftDirection(1);
            }

            runningOpMode.addTelemetry("Second Lift", "Can move freely", true);
        }
    }
}
