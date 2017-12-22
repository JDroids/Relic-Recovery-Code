package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.JDTeleopUsingRobot.getLiftDirection;
import static org.firstinspires.ftc.teamcode.JDTeleopUsingRobot.setLiftDirection;
import static org.firstinspires.ftc.teamcode.constants.DOWN;
import static org.firstinspires.ftc.teamcode.constants.FIRST_LIFT;
import static org.firstinspires.ftc.teamcode.constants.JEWEL_ARM_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.constants.JEWEL_KNOCKER_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.constants.SECOND_LIFT;
import static org.firstinspires.ftc.teamcode.constants.SERVO_GRABBER_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.constants.SERVO_GRABBER_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.constants.SERVO_GRABBER_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.constants.SERVO_GRABBER_WIDE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.constants.STRAFING_LIMIT;
import static org.firstinspires.ftc.teamcode.constants.UP;
import static org.firstinspires.ftc.teamcode.hardware.backLeftDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.backRightDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.firstGlyphLift;
import static org.firstinspires.ftc.teamcode.hardware.firstLiftSwitch;
import static org.firstinspires.ftc.teamcode.hardware.frontLeftDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.frontRightDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.jewelArm;
import static org.firstinspires.ftc.teamcode.hardware.jewelKnocker;
import static org.firstinspires.ftc.teamcode.hardware.secondGlyphLift;
import static org.firstinspires.ftc.teamcode.hardware.secondLiftSwitch;
import static org.firstinspires.ftc.teamcode.hardware.servoGrabberLeft;
import static org.firstinspires.ftc.teamcode.hardware.servoGrabberRight;

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
        if(!firstLiftSwitch.getState() && getLiftDirection(FIRST_LIFT) == UP){
            runningOpMode.addTelemetry("First Lift", "Top Limit Reached - Move Down", true);

            if (gamepad2.left_stick_y > 0) {
                //Move down at a slow speed as gravity is pulling it down
                firstGlyphLift.setPower(0.2);
                //LinearOpMode.class.wait allows sensor to move away from the magnet
                LinearOpMode.class.wait(200);
            }
            else {
                //Don't allow to move up any further
                firstGlyphLift.setPower(0);
            }
        }
        else if(!firstLiftSwitch.getState() && getLiftDirection(FIRST_LIFT) == DOWN){
            runningOpMode.addTelemetry("First Lift", "Bottom Limit Reached - Move Up", true);

            if(gamepad2.left_stick_y < 0){
                firstGlyphLift.setPower(-0.5);
                LinearOpMode.class.wait(400);
                setLiftDirection(FIRST_LIFT, UP);
            }
            else{
                //Don't allow to move down any further
                firstGlyphLift.setPower(0);
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
                setLiftDirection(FIRST_LIFT, UP);
            }
            else if(gamepad2.left_stick_y > 0){
                setLiftDirection(FIRST_LIFT, DOWN);
            }

            runningOpMode.addTelemetry("First Lift", "Can move freely", true);
        }
    }

    static public void secondLift(Gamepad gamepad2) throws InterruptedException{
        JDTeleopUsingRobot runningOpMode = new JDTeleopUsingRobot();

        if(!secondLiftSwitch.getState() && getLiftDirection(SECOND_LIFT) == UP) {
            runningOpMode.addTelemetry("Second Lift", "Top Limit Reached - Move Down", true);

            if (gamepad2.right_stick_y > 0) {
                //Move down at a slow speed as gravity is pulling it down
                secondGlyphLift.setPower(-0.3);

                LinearOpMode.class.wait(500);

                setLiftDirection(SECOND_LIFT, DOWN);
            }
            else {
                secondGlyphLift.setPower(0);
            }
        }
        else if(!secondLiftSwitch.getState() && getLiftDirection(SECOND_LIFT) == DOWN){
            runningOpMode.addTelemetry("Second Lift", "Bottom Limit Reached - Move Down", true);

            if(gamepad2.right_stick_y < 0){
                secondGlyphLift.setPower(0.5);
                LinearOpMode.class.wait(500);
                setLiftDirection(SECOND_LIFT, UP);
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
                setLiftDirection(SECOND_LIFT,UP);
            }
            else if(gamepad2.right_stick_y > 0){
                setLiftDirection(SECOND_LIFT,DOWN);
            }

            runningOpMode.addTelemetry("Second Lift", "Can move freely", true);
        }
    }

    static public void initDogeCV(HardwareMap hMap){
        CryptoboxDetector cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hMap.appContext, CameraViewDisplay.getInstance());

        cryptoboxDetector.downScaleFactor = 0.4;
        cryptoboxDetector.detectionMode = CryptoboxDetector.CryptoboxDetectionMode.HSV_RED; // Also HSV_BLUE for blue
        cryptoboxDetector.speed = CryptoboxDetector.CryptoboxSpeed.BALANCED;
        cryptoboxDetector.rotateMat = true;

        cryptoboxDetector.enable();
    }

    static public void lowerJewelArms(){
        redRecoveryAutoDogeCV runningOpMode = new redRecoveryAutoDogeCV();

        jewelKnocker.setPosition(0.5);
        runningOpMode.sleep(200);

        jewelArm.setPosition(0.05);
        runningOpMode.sleep(200);
    }

    /*static public int detectJewelColor(){
        //1 is Red, 2 is Blue

        int jewelColorFound = 0;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while(!(runtime.seconds() > 3 || jewelColorFound != 0)){

        }

    }*/
}
