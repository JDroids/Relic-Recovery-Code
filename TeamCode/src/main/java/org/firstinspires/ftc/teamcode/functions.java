package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.JDTeleopUsingRobot.getLiftDirection;
import static org.firstinspires.ftc.teamcode.JDTeleopUsingRobot.setLiftDirection;
import static org.firstinspires.ftc.teamcode.constants.DOWN;
import static org.firstinspires.ftc.teamcode.constants.FIRST_LIFT;
import static org.firstinspires.ftc.teamcode.constants.JEWEL_ARM_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.constants.JEWEL_KNOCKER_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.constants.JewelColor;
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
import static org.firstinspires.ftc.teamcode.hardware.frontRangeSensor;
import static org.firstinspires.ftc.teamcode.hardware.frontRightDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.imuSensor;
import static org.firstinspires.ftc.teamcode.hardware.jewelArm;
import static org.firstinspires.ftc.teamcode.hardware.jewelColorSensor;
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

    static public void turn(int degrees){
        countPasses runningOpMode = new countPasses();

        Orientation angles;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imuSensor.initialize(parameters);

        angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        float originalZ = angles.firstAngle;
        float currentZ = angles.firstAngle;

        while((!(currentZ >= degrees-3) && !(currentZ<=degrees+3)) && runningOpMode.opModeIsActive()){
            angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            currentZ = angles.firstAngle;

            if(degrees > 0){
                frontLeftDriveMotor.setPower(0.3);
                frontRightDriveMotor.setPower(0.3);
                backLeftDriveMotor.setPower(0.3);
                backRightDriveMotor.setPower(0.3);
            }
            else{
                frontLeftDriveMotor.setPower(-0.3);
                frontRightDriveMotor.setPower(-0.3);
                backLeftDriveMotor.setPower(-0.3);
                backRightDriveMotor.setPower(-0.3);
            }

            runningOpMode.telemetry.addData("Current Z", currentZ);
            runningOpMode.telemetry.addData("Original Z", originalZ);
        }

        frontLeftDriveMotor.setPower(0.0);
        frontRightDriveMotor.setPower(0.0);
        backLeftDriveMotor.setPower(0.0);
        backRightDriveMotor.setPower(0.0);

    }

    static public JewelColor detectJewelColor(){

        JewelColor jewelColorFound = JewelColor.NONE;

        long startTime = System.nanoTime();
        long estimatedTime = System.nanoTime() - startTime;

        while( TimeUnit.NANOSECONDS.toSeconds(estimatedTime) < 3 ||
                jewelColorFound == JewelColor.RED || jewelColorFound == JewelColor.BLUE){

            // hsvValues is an array that will hold the hue, saturation, and value information.
            float hsvValues[] = {0F, 0F, 0F};

            // values is a reference to the hsvValues array.
            final float values[] = hsvValues;
            // sometimes it helps to multiply the raw RGB values with a scale factor
            // to amplify/attentuate the measured values.
            final double SCALE_FACTOR = 255;
            jewelColorSensor.enableLed(true);
                // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (jewelColorSensor.red() * SCALE_FACTOR), (int) (jewelColorSensor.green() * SCALE_FACTOR),
                    (int) (jewelColorSensor.blue() * SCALE_FACTOR),hsvValues);
            //TODO: based on hue value determine the color - look at Blocks code for the actual values
            float hue = hsvValues[0];
            if (hue >= 190 && hue <= 235) {
                jewelColorFound = JewelColor.BLUE;
            }

            else if (hue <= 15 || hue >= 350) {
                jewelColorFound = JewelColor.RED;

            }

            else if (hue == 0); {
                jewelColorFound = JewelColor.NONE;

            }

        }
        return jewelColorFound;
    }

    //get five readings to sample
    static public double[]  readDistanceFromPerimeterWallUsingRange(){

       double[] sensorValues = new double[5];
       for ( int i=0 ; i< 5 ; i++ ){
           sensorValues[i] = frontRangeSensor.getDistance(DistanceUnit.CM);
           //TODO: should we wait for 100 ms before we take the next reading..
       }
       return sensorValues;
    }

    //AVT Algorithm to filter range sensor values and return the sampled distance
    // calculate average value
    //AVT algorithm stands for Antonyan Vardan Transform and its implementation explained below.
    //Collect n samples of data
    //Calculate the standard deviation and average value
    //Drop any data that is greater or less than average ± one standard deviation
    //Calculate average value of remaining data
    static double filterRangeSensorValues(double[] rangeSensorValues){

        //calculate average
        double sum = 0;
        for(int i=0; i < rangeSensorValues.length ; i++)
            sum = sum + rangeSensorValues[i];
        double average = sum / rangeSensorValues.length;

        //calculate standard deviation
        double sd = 0;
        for(int i = 0; i < rangeSensorValues.length; i++){
            sd = sd + Math.pow((rangeSensorValues[i]-average),2);
        }
        double standardDeviation = Math.sqrt(sd/rangeSensorValues.length);

        //Drop any data that is greater or less than average ± one standard deviation
        int newlength=0;
        double newSum=0;
        for(int i = 0; i < rangeSensorValues.length; i++){
            if ( !(rangeSensorValues[i] > average+standardDeviation || rangeSensorValues[i] < average-standardDeviation ) ){
                newlength++;
                newSum= newSum+rangeSensorValues[i];
            }
        }

        //distance is average value of remaining data
        return newSum/newlength;
    }

}
