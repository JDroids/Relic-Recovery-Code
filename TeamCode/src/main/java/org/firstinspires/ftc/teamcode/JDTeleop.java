package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by dansm on 12/7/2017.
 */

@TeleOp(name="JDTeleop")

public class JDTeleop extends LinearOpMode{
    @Override

    public void runOpMode() throws InterruptedException{
        //Code to run after init is pressed

        double STRAFING_LIMIT = 0.1;

        double[] SERVO_GRABBER_INIT_POSITION = [double 0.5, double 0.3];
        double[] SERVO_GRABBER_CLOSE_POSITION = [double 0.0, double 0.8];
        double[] SERVO_GRABBER_OPEN_POSITION = [double 0.2, double 0.6];
        double[] SERVO_GRABBER_WIDE_OPEN_POSITION = [double 0.5, double 0.3];


        double JEWEL_KNOCK_INIT_POSITION = 0;
        double JEWEL_ARM_INIT_POSITION = 0.9;



        DcMotor frontLeftDriveMotor = null;
        DcMotor frontRightDriveMotor = null;
        DcMotor backLeftDriveMotor = null;
        DcMotor backRightDriveMotor = null;

        DcMotor firstGlyphLift = null;
        DcMotor secondGlyphLift = null;

        Servo servoGrabberLeft = null;
        Servo servoGrabberRight = null;

        //TODO: Get hardware map setup

        frontLeftDriveMotor = hardwareMap.dcMotor.get("FrontLeft");
        frontRightDriveMotor = hardwareMap.dcMotor.get("FrontRight");
        backLeftDriveMotor = hardwareMap.dcMotor.get("BackLeft");
        backRightDriveMotor = hardwareMap.dcMotor.get("BackRight");

        firstGlyphLift = hardwareMap.dcMotor.get("MotorGlyphLift");
        secondGlyphLift = hardwareMap.dcMotor.get("MotorGlyphLift2");

        servoGrabberLeft = hardwareMap.servo.get("servoGrabberRight");
        servoGrabberRight = hardwareMap.servo.get("servoGrabberLeft");

        waitForStart();

        setGrabber(SERVO_GRABBER_INIT_POSITION[0], SERVO_GRABBER_INIT_POSITION[1]);

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

            if(gamepad2.a){
                setGrabber(SERVO_GRABBER_CLOSE_POSITION[0], SERVO_GRABBER_CLOSE_POSITION[1]);
            }
            else if(gamepad2.b){
                setGrabber(SERVO_GRABBER_OPEN_POSITION[0], SERVO_GRABBER_OPEN_POSITION[1]);
            }
            else if(gamepad2.y){
                setGrabber(SERVO_GRABBER_WIDE_OPEN_POSITION[0], SERVO_GRABBER_WIDE_OPEN_POSITION[1]);
            }

            //TODO: Get glyph grabber lift and closing/opening setup NOTE: SERVOS CAN'T MOVE DURING TELEOP INIT THAT IS ILLEGAL
        }

        //Code to run after play is pressed
    }

    public void move(double leftY, double rightY, double leftX, double rightX){
        if(leftY >= STRAFING_LIMIT && rightY >= STRAFING_LIMIT || leftX <= STRAFING_LIMIT && rightX <= STRAFING_LIMIT){
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

    public void setGrabber(double leftServoPosition, double rightServoPosition){
        servoGrabberLeft.setPosition(leftServoPosition);
        servoGrabberRight.setPosition(rightServoPosition);
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
            dScale = -scaleArray[index];
        }
        else{
            dScale = scaleArray[index];
        }

        return dScale;

    }
}