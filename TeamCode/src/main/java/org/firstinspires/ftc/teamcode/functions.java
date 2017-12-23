package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.JDTeleopUsingRobot.getLiftDirection;
import static org.firstinspires.ftc.teamcode.JDTeleopUsingRobot.setLiftDirection;
import static org.firstinspires.ftc.teamcode.constants.BOTH_GRABBERS;
import static org.firstinspires.ftc.teamcode.constants.BOTTOM_GRABBER;
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
import static org.firstinspires.ftc.teamcode.constants.TOP_GRABBER;
import static org.firstinspires.ftc.teamcode.constants.UP;
import static org.firstinspires.ftc.teamcode.hardware.backLeftDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.backRightDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.firstGlyphLift;
import static org.firstinspires.ftc.teamcode.hardware.firstLiftSwitch;
import static org.firstinspires.ftc.teamcode.hardware.frontLeftDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.frontRangeSensor;
import static org.firstinspires.ftc.teamcode.hardware.frontRightDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.glyphGrabberBL;
import static org.firstinspires.ftc.teamcode.hardware.glyphGrabberBR;
import static org.firstinspires.ftc.teamcode.hardware.glyphGrabberTL;
import static org.firstinspires.ftc.teamcode.hardware.glyphGrabberTR;
import static org.firstinspires.ftc.teamcode.hardware.imuSensor;
import static org.firstinspires.ftc.teamcode.hardware.jewelArm;
import static org.firstinspires.ftc.teamcode.hardware.jewelColorSensor;
import static org.firstinspires.ftc.teamcode.hardware.jewelKnocker;
import static org.firstinspires.ftc.teamcode.hardware.secondGlyphLift;
import static org.firstinspires.ftc.teamcode.hardware.secondLiftSwitch;
import static org.firstinspires.ftc.teamcode.hardware.sideRangeSensor;

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

    static public void setGrabber(double leftServoPosition, double rightServoPosition, int grabbers) throws InterruptedException{
        if(grabbers == BOTH_GRABBERS){
            glyphGrabberTL.setPosition(leftServoPosition);
            glyphGrabberTR.setPosition(rightServoPosition);
            glyphGrabberBL.setPosition(leftServoPosition);
            glyphGrabberBR.setPosition(rightServoPosition);
        }
        else if(grabbers == BOTTOM_GRABBER){
            glyphGrabberBL.setPosition(leftServoPosition);
            glyphGrabberBR.setPosition(rightServoPosition);
        }
        else if(grabbers == TOP_GRABBER){
            glyphGrabberTL.setPosition(leftServoPosition);
            glyphGrabberTR.setPosition(rightServoPosition);
        }
    }

    static public void setJewelPosition(double jewelKnockerPosition, double jewelArmPosition){
        jewelKnocker.setPosition(jewelKnockerPosition);
        jewelArm.setPosition(jewelArmPosition);
    }

    static public void initServos() throws InterruptedException{
        setGrabber(SERVO_GRABBER_INIT_POSITION[0], SERVO_GRABBER_INIT_POSITION[1], BOTH_GRABBERS);
        setJewelPosition(JEWEL_KNOCKER_INIT_POSITION, JEWEL_ARM_INIT_POSITION);
    }

    static public void closeGrabber(int grabbers) throws InterruptedException{
        setGrabber(SERVO_GRABBER_CLOSE_POSITION[0], SERVO_GRABBER_CLOSE_POSITION[1], grabbers);
    }

    static public void openGrabber(int grabbers) throws InterruptedException{
        setGrabber(SERVO_GRABBER_OPEN_POSITION[0], SERVO_GRABBER_OPEN_POSITION[1], grabbers);
    }

    static public void openGrabberWide(int grabbers) throws InterruptedException{
        setGrabber(SERVO_GRABBER_WIDE_OPEN_POSITION[0], SERVO_GRABBER_WIDE_OPEN_POSITION[1], grabbers);
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

    static public void stop(){
        frontLeftDriveMotor.setPower(0);
        frontRightDriveMotor.setPower(0);
        backLeftDriveMotor.setPower(0);
        backRightDriveMotor.setPower(0);
    }

    static public void moveForTime(double power, int milliseconds, LinearOpMode linearOpMode){
        frontLeftDriveMotor.setPower(-power);
        frontRightDriveMotor.setPower(power);
        backLeftDriveMotor.setPower(-power);
        backRightDriveMotor.setPower(power);

        linearOpMode.sleep(milliseconds);
        stop();
    }

    static public void firstLift(Gamepad gamepad2, LinearOpMode linearOpMode) throws InterruptedException{
        if(!firstLiftSwitch.getState() && getLiftDirection(FIRST_LIFT) == UP){
            linearOpMode.telemetry.addData("First Lift", "Top Limit Reached - Move Down");
            linearOpMode.telemetry.update();

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
            linearOpMode.telemetry.addData("First Lift", "Bottom Limit Reached - Move Up");
            linearOpMode.telemetry.update();

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

            linearOpMode.telemetry.addData("First Lift", "Can move freely", true);
            linearOpMode.telemetry.update();
        }
    }

    static public void secondLift(Gamepad gamepad2, LinearOpMode linearOpMode) throws InterruptedException{
        if(!secondLiftSwitch.getState() && getLiftDirection(SECOND_LIFT) == UP) {
            linearOpMode.telemetry.addData("Second Lift", "Top Limit Reached - Move Down");
            linearOpMode.telemetry.update();

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
            linearOpMode.telemetry.addData("Second Lift", "Bottom Limit Reached - Move Down");
            linearOpMode.telemetry.update();

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

            linearOpMode.telemetry.addData("Second Lift", "Can move freely");
            linearOpMode.telemetry.update();
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

    static public void lowerJewelArms(LinearOpMode linearOpMode){
        jewelKnocker.setPosition(0.5);
        linearOpMode.sleep(200);

        jewelArm.setPosition(0.05);
        linearOpMode.sleep(200);
    }

    static public void turn(int degrees, LinearOpMode linearOpMode){
        Orientation angles;

        angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
        float originalZ = angles.firstAngle;
        float currentZ = angles.firstAngle;

        linearOpMode.telemetry.addData("Current Z", currentZ);
        linearOpMode.telemetry.addData("Original Z", originalZ);
        linearOpMode.telemetry.update();

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

        while((!(currentZ >= degrees - 5) && (currentZ <= degrees + 5)) && linearOpMode.opModeIsActive()){
            angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
            currentZ = angles.firstAngle;

            linearOpMode.telemetry.addData("Current Z", currentZ);
            linearOpMode.telemetry.addData("Original Z", originalZ);
            linearOpMode.telemetry.update();
        }

        stop();
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

    static public RelicRecoveryVuMark getVumark(LinearOpMode linearOpMode, HardwareMap hMap){
        VuforiaLocalizer vuforia;
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

        int cameraMonitorViewId = hMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = " AZcIMlr/////AAAAGe1W/L9P20hXupxJsIH5bIMDl46JPwjrX2kI+L6+tigIG9bhthzvrEWVBni6g4Jkvs76N/hIT0bFun78pnNDqkG3ZP24XLj45VHA2rYKp8UDww/vfy8xrtvHxedihdX1A2vMWg8Ub8tLjBMgEAqcAYYUMwPRQfI61KQmXvAJBV79XtQughxCh/fbrtoux6WV6HHs8OydP7kPUaUU3f0z5ZOF/TUvcqFFotqnLg/KwXMxxrouRyDGCIbpbP7cYabiR7ShIGvrYoRKtbpwxS3WLSjjTd7ynvoidYipWZ60e6t+wUCzdXahS8g0veYuTQ+vwBqljhtLUWnCUjbJh2jocjxV9kLGgqlPFCmLHZyurYkX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); //For debug purposes
        //Works here
        //runningOpMode.addTelemetry("VuMark Detection State", "Starting", true);

        relicTrackables.activate();

        long startTime = System.nanoTime();
        long estimatedTime = System.nanoTime() - startTime;

        //try to read the vumark until we find a valid vumark or for 3 seconds
        while(vuMark == RelicRecoveryVuMark.UNKNOWN  || TimeUnit.NANOSECONDS.toSeconds(estimatedTime) <= 3 ){
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN){
                linearOpMode.telemetry.addData("Vumark Found", vuMark.toString());
                linearOpMode.telemetry.update();
                break;
            }
        }

        if (vuMark == RelicRecoveryVuMark.UNKNOWN){
            //if more than 3 seconds and vumark is not found, default to LEFT
            return RelicRecoveryVuMark.LEFT;
        }

        return vuMark;

    }


    static public void moveUntilCryptoWall(double distanceToWall, RelicRecoveryVuMark vuMark, LinearOpMode linearOpMode){
        int targetColumn;

        if(vuMark == RelicRecoveryVuMark.LEFT){
            targetColumn = 1;
        }
        else if(vuMark == RelicRecoveryVuMark.CENTER){
            targetColumn = 2;
        }
        else if(vuMark == RelicRecoveryVuMark.RIGHT){
            targetColumn = 3;
        }
        else{
            targetColumn = 1;
        }

        int columnsPassed = 0;

        double distanceToCryptoBoxWall = distanceToWall - 5;

        frontLeftDriveMotor.setPower(0.2);
        frontRightDriveMotor.setPower(-0.2);
        backLeftDriveMotor.setPower(0.2);
        backRightDriveMotor.setPower(-0.2);
        while(linearOpMode.opModeIsActive()) {
            if (sideRangeSensor.cmUltrasonic() <= distanceToCryptoBoxWall) {
                columnsPassed++;

                while (sideRangeSensor.cmUltrasonic() <= distanceToCryptoBoxWall && linearOpMode.opModeIsActive()) {
                }
            }
            if (columnsPassed >= targetColumn) {
                frontLeftDriveMotor.setPower(0);
                frontRightDriveMotor.setPower(0);
                backLeftDriveMotor.setPower(0);
                backRightDriveMotor.setPower(0);
                break;
            }

            linearOpMode.telemetry.addData("Distance to Wall", distanceToWall);
            linearOpMode.telemetry.addData("Distance to Crypto Box Wall", distanceToCryptoBoxWall);
            linearOpMode.telemetry.addData("Centimeters from Object", sideRangeSensor.cmUltrasonic());
            linearOpMode.telemetry.addData("Columns Passed", columnsPassed);
            linearOpMode.telemetry.update();
        }
    }

    //AVT Algorithm to filter range sensor values and return the sampled distance
    // calculate average value
    //AVT algorithm stands for Antonyan Vardan Transform and its implementation explained below.
    //Collect n samples of data
    //Calculate the standard deviation and average value
    //Drop any data that is greater or less than average ± one standard deviation
    //Calculate average value of remaining data

    static public double filterRangeSensorValues(double[] rangeSensorValues){

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
