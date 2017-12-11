package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by dansm on 12/10/2017.
 */

@Disabled

public class robot{

    //Constants

    double STRAFING_LIMIT = 0.1;

    double[] SERVO_GRABBER_INIT_POSITION = new double[]{0.5, 0.3};
    double[] SERVO_GRABBER_CLOSE_POSITION = new double[]{0.0, 0.8};
    double[] SERVO_GRABBER_OPEN_POSITION = new double[]{0.2, 0.6};
    double[] SERVO_GRABBER_WIDE_OPEN_POSITION = new double[]{0.5, 0.3};

    double JEWEL_KNOCKER_INIT_POSITION = 0;
    double JEWEL_ARM_INIT_POSITION = 0.9;


    //Hardware

    DcMotor frontLeftDriveMotor = hardwareMap.dcMotor.get("FrontLeft");
    DcMotor frontRightDriveMotor = hardwareMap.dcMotor.get("FrontRight");
    DcMotor backLeftDriveMotor = hardwareMap.dcMotor.get("BackLeft");
    DcMotor backRightDriveMotor = hardwareMap.dcMotor.get("BackRight");

    DcMotor firstGlyphLift = hardwareMap.dcMotor.get("MotorGlyphLift");
    DcMotor secondGlyphLift = hardwareMap.dcMotor.get("MotorGlyphLift2");

    Servo servoGrabberLeft = hardwareMap.servo.get("servoGrabberRight");
    Servo servoGrabberRight = hardwareMap.servo.get("servoGrabberLeft");

    Servo jewelKnocker = hardwareMap.servo.get("servoJewelKnock");
    Servo jewelArm = hardwareMap.servo.get("servoJewelArm");

    DigitalChannel firstLiftSwitch = hardwareMap.digitalChannel.get("FirstLiftSwitch");
    DigitalChannel secondLiftSwitch = hardwareMap.digitalChannel.get("SecondLiftSwitch");





    //Functions

    public double scaleInput(double dVal) throws InterruptedException {
        double result = Math.pow(dVal, 3);
        if (result > 0.7) {
            result = 0.7;
        }
        else if (result < -0.7) {
            result = -0.7;
        }
        return result;
    }

    public void initServos() throws InterruptedException {
        setGrabber(SERVO_GRABBER_INIT_POSITION[0], SERVO_GRABBER_INIT_POSITION[1]);
        setJewelPosition(JEWEL_KNOCKER_INIT_POSITION, JEWEL_ARM_INIT_POSITION);
    }
}