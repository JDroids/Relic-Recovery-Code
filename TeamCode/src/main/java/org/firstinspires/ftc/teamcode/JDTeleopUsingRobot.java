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

        double gamepad1LeftY;
        double gamepad1RightY;
        double gamepad1LeftX;
        double gamepad1RightX;

        waitForStart();
        //Code to run after play is pressed

        initServos();

        firstLiftSwitch.setMode(DigitalChannel.Mode.Input);
        secondLiftSwitch.setMode(DigitalChannel.Mode.Input);

        while(opModeIsActive()){
            gamepad1LeftY = scaleInput(gamepad1.left_stick_y);
            gamepad1RightY = scaleInput(gamepad1.right_stick_y);
            gamepad1LeftX = scaleInput(gamepad1.left_stick_x);
            gamepad1RightX = scaleInput(gamepad1.right_stick_x);

            move(gamepad1LeftY, gamepad1RightY, gamepad1LeftX, gamepad1RightX);

            if(gamepad2.a){
                setGrabber(SERVO_GRABBER_CLOSE_POSITION[0], SERVO_GRABBER_CLOSE_POSITION[1]);
            }
            else if(gamepad2.b){
                setGrabber(SERVO_GRABBER_OPEN_POSITION[0], SERVO_GRABBER_OPEN_POSITION[1]);
            }
            else if(gamepad2.y){
                setGrabber(SERVO_GRABBER_WIDE_OPEN_POSITION[0], SERVO_GRABBER_WIDE_OPEN_POSITION[1]);
            }

            firstLift();
            secondLift();
        }


    }

    public void move(double leftY, double rightY, double leftX, double rightX) throws InterruptedException{
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

    public void setGrabber(double leftServoPosition, double rightServoPosition) throws InterruptedException{
        servoGrabberLeft.setPosition(leftServoPosition);
        servoGrabberRight.setPosition(rightServoPosition);
    }

    public void setJewelPosition(double jewelKnockerPosition, double jewelArmPosition){
        jewelKnocker.setPosition(jewelKnockerPosition);
        jewelArm.setPosition(jewelArmPosition);
    }



    public void firstLift() throws InterruptedException{
        if(!firstLiftSwitch.getState() && firstLiftDirection == 1) {
            telemetry.addData("First Lift", "Top Limit Reached - Move Down");
            telemetry.update();

            if (gamepad2.left_stick_y > 0) {
                //Move down at a slow speed as gravity is pulling it down
                firstGlyphLift.setPower(0.2);

                //Sleep allows sensor to move away from the magnet
                Thread.sleep(200);

                firstLiftDirection = -1;
            } else {
                firstGlyphLift.setPower(0);
            }
        }
        else if(!firstLiftSwitch.getState() && firstLiftDirection == -1){
            telemetry.addData("First Lift", "Bottom Limit Reached - Move Up");
            telemetry.update();

            if(gamepad2.left_stick_y < 0){
                secondGlyphLift.setPower(-0.5);

                Thread.sleep(400);

                firstLiftDirection = 1;
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
                firstLiftDirection = 1;
            }
            else if(gamepad2.left_stick_y > 0){
                firstLiftDirection = -1;
            }

            telemetry.addData("First Lift", "Can move freely");
            telemetry.update();
        }
    }

    public void secondLift() throws InterruptedException    {
        if(!secondLiftSwitch.getState() && secondLiftDirection == 1) {
            telemetry.addData("Second Lift", "Top Limit Reached - Move Down");
            telemetry.update();

            if (gamepad2.right_stick_y > 0) {
                //Move down at a slow speed as gravity is pulling it down
                secondGlyphLift.setPower(-0.3);

                Thread.sleep(500);

                secondLiftDirection = -1;
            } else {
                secondGlyphLift.setPower(0);
            }
        }
        else if(!secondLiftSwitch.getState() && secondLiftDirection == -1){
            telemetry.addData("Second Lift", "Bottom Limit Reached - Move Down");
            telemetry.update();

            if(gamepad2.right_stick_y < 0){
                secondGlyphLift.setPower(0.5);

                Thread.sleep(500);

                secondLiftDirection = 1;
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
                secondLiftDirection = 1;
            }
            else if(gamepad2.right_stick_y > 0){
                secondLiftDirection = -1;
            }

            telemetry.addData("Second Lift", "Can move freely");
            telemetry.update();
        }
    }
}