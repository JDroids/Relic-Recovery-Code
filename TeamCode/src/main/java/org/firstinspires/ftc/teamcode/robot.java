package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by dansm on 12/10/2017.
 */

@Disabled

public class robot{

    //Constants

    static double STRAFING_LIMIT = 0.1;
    static double[] SERVO_GRABBER_INIT_POSITION = new double[]{0.5, 0.3};
    static double[] SERVO_GRABBER_CLOSE_POSITION = new double[]{0.0, 0.8};
    static double[] SERVO_GRABBER_OPEN_POSITION = new double[]{0.2, 0.6};
    static double[] SERVO_GRABBER_WIDE_OPEN_POSITION = new double[]{0.5, 0.3};

    static double JEWEL_KNOCKER_INIT_POSITION = 0;
    static double JEWEL_ARM_INIT_POSITION = 0.9;


    //Hardware
    static DcMotor frontLeftDriveMotor = null;
    static DcMotor frontRightDriveMotor = null;
    static DcMotor backLeftDriveMotor = null;
    static DcMotor backRightDriveMotor = null;

    static DcMotor firstGlyphLift = null;
    static DcMotor secondGlyphLift = null;

    static Servo servoGrabberLeft = null;
    static Servo servoGrabberRight = null;

    static Servo jewelKnocker = null;
    static Servo jewelArm = null;

    static DigitalChannel firstLiftSwitch = null;
    static DigitalChannel secondLiftSwitch = null;

    static public boolean initHardwareMap(HardwareMap map){
        HardwareMap hMap = map;

        try{
            frontLeftDriveMotor = hMap.dcMotor.get("FrontLeft");
            frontRightDriveMotor = hMap.dcMotor.get("FrontRight");
            backLeftDriveMotor = hMap.dcMotor.get("BackLeft");
            backRightDriveMotor = hMap.dcMotor.get("BackRight");

            firstGlyphLift = hMap.dcMotor.get("MotorGlyphLift");
            secondGlyphLift = hMap.dcMotor.get("MotorGlyphLift2");

            servoGrabberLeft = hMap.servo.get("servoGrabberRight");
            servoGrabberRight = hMap.servo.get("servoGrabberLeft");

            jewelKnocker = hMap.servo.get("servoJewelKnock");
            jewelArm = hMap.servo.get("servoJewelArm");

            firstLiftSwitch = hMap.digitalChannel.get("FirstLiftSwitch");
            secondLiftSwitch = hMap.digitalChannel.get("SecondLiftSwitch");

            firstLiftSwitch.setMode(DigitalChannel.Mode.INPUT);
            secondLiftSwitch.setMode(DigitalChannel.Mode.INPUT);

            return true;
        }
        catch(Exception e){
            return false;
        }
    }

    //Functions

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

    static public void wideOpenGrabber() throws InterruptedException{
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

    static public int firstLift(int liftDirection) throws InterruptedException{
        if(!firstLiftSwitch.getState() && liftDirection == 1){
            telemetry.addData("First Lift", "Top Limit Reached - Move Down");
            telemetry.update();

            if (gamepad2.left_stick_y > 0) {
                //Move down at a slow speed as gravity is pulling it down
                firstGlyphLift.setPower(0.2);

                //Sleep allows sensor to move away from the magnet
                Thread.sleep(200);

                return -1;
            } else {
                firstGlyphLift.setPower(0);
            }
        }
        else if(!firstLiftSwitch.getState() && liftDirection == -1){
            telemetry.addData("First Lift", "Bottom Limit Reached - Move Up");
            telemetry.update();

            if(gamepad2.left_stick_y < 0){
                secondGlyphLift.setPower(-0.5);

                Thread.sleep(400);

                return 1;
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
                return 1;
            }
            else if(gamepad2.left_stick_y > 0){
                return -1;
            }

            telemetry.addData("First Lift", "Can move freely");
            telemetry.update();
        }
    }

    static public int secondLift() throws InterruptedException{
        if(!secondLiftSwitch.getState() && liftDirection == 1) {
            telemetry.addData("Second Lift", "Top Limit Reached - Move Down");
            telemetry.update();

            if (gamepad2.right_stick_y > 0) {
                //Move down at a slow speed as gravity is pulling it down
                secondGlyphLift.setPower(-0.3);

                Thread.sleep(500);

                return -1;
            } else {
                secondGlyphLift.setPower(0);
            }
        }
        else if(!secondLiftSwitch.getState() && liftDirection == -1){
            telemetry.addData("Second Lift", "Bottom Limit Reached - Move Down");
            telemetry.update();

            if(gamepad2.right_stick_y < 0){
                secondGlyphLift.setPower(0.5);

                Thread.sleep(500);

                return 1;
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
                return 1;
            }
            else if(gamepad2.right_stick_y > 0){
                return -1;
            }

            telemetry.addData("Second Lift", "Can move freely");
            telemetry.update();
        }
    }

}