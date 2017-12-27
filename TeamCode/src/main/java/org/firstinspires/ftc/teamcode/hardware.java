
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by dansm on 12/13/2017.
 */

public class hardware{
    static DcMotor frontLeftDriveMotor = null;
    static DcMotor frontRightDriveMotor = null;
    static DcMotor backLeftDriveMotor = null;
    static DcMotor backRightDriveMotor = null;

    static DcMotor firstGlyphLift = null;
    static DcMotor secondGlyphLift = null;

    static Servo servoGrabberLeft = null;
    static Servo servoGrabberRight = null;

    static Servo glyphGrabberTL = null;
    static Servo glyphGrabberTR = null;
    static Servo glyphGrabberBL = null;
    static Servo glyphGrabberBR = null;

    static Servo jewelKnocker = null;
    static Servo jewelArm = null;

    static DigitalChannel firstLiftSwitch = null;
    static DigitalChannel secondLiftSwitch = null;

    static ColorSensor jewelColorSensor = null;
    static ModernRoboticsI2cRangeSensor sideRangeSensor;
    static ModernRoboticsI2cRangeSensor frontRangeSensor;
    static BNO055IMU imuSensor = null;

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

            glyphGrabberTL = hMap.servo.get("lowerServoGrabberLeft");
            glyphGrabberTR = hMap.servo.get("lowerServoGrabberRight");
            glyphGrabberBL = hMap.servo.get("servoGrabberLeft ");
            glyphGrabberBR = hMap.servo.get("servoGrabberRight");

            jewelKnocker = hMap.servo.get("servoJewelKnock");
            jewelArm = hMap.servo.get("servoJewelArm");

            firstLiftSwitch = hMap.digitalChannel.get("FirstLiftSwitch");
            secondLiftSwitch = hMap.digitalChannel.get("SecondLiftSwitch");

            firstLiftSwitch.setMode(DigitalChannel.Mode.INPUT);
            secondLiftSwitch.setMode(DigitalChannel.Mode.INPUT);

            imuSensor = hMap.get(BNO055IMU.class, "imu");

            jewelColorSensor = hMap.colorSensor.get("color1");

            //frontRangeSensor = hMap.get(ModernRoboticsI2cRangeSensor.class, "frontRange");
            sideRangeSensor = hMap.get(ModernRoboticsI2cRangeSensor.class, "LeftRange");


            return true;
        }
        catch(Exception e){
            return false;
        }
    }
}