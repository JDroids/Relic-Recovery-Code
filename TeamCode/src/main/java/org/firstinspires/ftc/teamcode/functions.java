package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.source.tree.ForLoopTree;

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
import static org.firstinspires.ftc.teamcode.constants.BOTTOM_SERVO_GRABBER_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.constants.DOWN;
import static org.firstinspires.ftc.teamcode.constants.FIRST_LIFT;
import static org.firstinspires.ftc.teamcode.constants.JEWEL_ARM_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.constants.JEWEL_KNOCKER_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.constants.JDColor;
import static org.firstinspires.ftc.teamcode.constants.SECOND_LIFT;
import static org.firstinspires.ftc.teamcode.constants.TOP_SERVO_GRABBER_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.constants.BOTTOM_SERVO_GRABBER_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.constants.TOP_SERVO_GRABBER_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.constants.BOTTOM_SERVO_GRABBER_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.constants.TOP_SERVO_GRABBER_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.constants.BOTTOM_SERVO_GRABBER_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.constants.TOP_SERVO_GRABBER_WIDE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.constants.BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION;

import static org.firstinspires.ftc.teamcode.constants.STRAFING_LIMIT;
import static org.firstinspires.ftc.teamcode.constants.TOP_GRABBER;
import static org.firstinspires.ftc.teamcode.constants.UP;;
import static org.firstinspires.ftc.teamcode.hardware.backLeftDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.backRightDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.firstGlyphLift;
import static org.firstinspires.ftc.teamcode.hardware.firstLiftSwitch;
import static org.firstinspires.ftc.teamcode.hardware.frontLeftDriveMotor;
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
import static org.firstinspires.ftc.teamcode.hardware.rearRangeSensor;

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

    //scaling logic 2 to use 3 fixed speeds as opposed to varying speeds to avoid jerks while driving
    static public double scaleInputFixedSpeed(double dVal) throws InterruptedException {
        int sign = (int) (dVal/ Math.abs(dVal));
        double result = Math.abs(dVal);

        if ( result < 0.4 ) {
            result = 0.3;
        }
        else if (result < 0.7) {
            result = 0.6;
        }
        else {
            result = 0.75;
        }
        return result*sign ;
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
        setGrabber(TOP_SERVO_GRABBER_INIT_POSITION[0], TOP_SERVO_GRABBER_INIT_POSITION[1], TOP_GRABBER);
        setGrabber(BOTTOM_SERVO_GRABBER_INIT_POSITION[0], BOTTOM_SERVO_GRABBER_INIT_POSITION[1], BOTTOM_GRABBER);
        setJewelPosition(JEWEL_KNOCKER_INIT_POSITION, JEWEL_ARM_INIT_POSITION);
    }

    static public void closeGrabber(int grabber) throws InterruptedException{
        if (grabber == BOTH_GRABBERS){
            setGrabber(TOP_SERVO_GRABBER_CLOSE_POSITION[0], TOP_SERVO_GRABBER_CLOSE_POSITION[1], TOP_GRABBER);
            setGrabber(BOTTOM_SERVO_GRABBER_CLOSE_POSITION[0], BOTTOM_SERVO_GRABBER_CLOSE_POSITION[1], BOTTOM_GRABBER);
        }else if ( grabber == TOP_GRABBER){
            setGrabber(TOP_SERVO_GRABBER_CLOSE_POSITION[0], TOP_SERVO_GRABBER_CLOSE_POSITION[1], TOP_GRABBER);

        }else if (grabber == BOTTOM_GRABBER){
            setGrabber(BOTTOM_SERVO_GRABBER_CLOSE_POSITION[0], BOTTOM_SERVO_GRABBER_CLOSE_POSITION[1], BOTTOM_GRABBER);
        }
    }

    static public void openGrabber(int grabber) throws InterruptedException{
        if (grabber == BOTH_GRABBERS){
            setGrabber(TOP_SERVO_GRABBER_OPEN_POSITION[0], TOP_SERVO_GRABBER_OPEN_POSITION[1], TOP_GRABBER);
            setGrabber(BOTTOM_SERVO_GRABBER_OPEN_POSITION[0], BOTTOM_SERVO_GRABBER_OPEN_POSITION[1], BOTTOM_GRABBER);
        }else if ( grabber == TOP_GRABBER){
            setGrabber(TOP_SERVO_GRABBER_OPEN_POSITION[0], TOP_SERVO_GRABBER_OPEN_POSITION[1], TOP_GRABBER);

        }else if (grabber == BOTTOM_GRABBER){
            setGrabber(BOTTOM_SERVO_GRABBER_OPEN_POSITION[0], BOTTOM_SERVO_GRABBER_OPEN_POSITION[1], BOTTOM_GRABBER);
        }
    }

    static public void openGrabberWide(int grabber) throws InterruptedException{
        if (grabber == BOTH_GRABBERS){
            setGrabber(TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[0], TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[1], TOP_GRABBER);
            setGrabber(BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[0], BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[1], BOTTOM_GRABBER);
        }else if ( grabber == TOP_GRABBER){
            setGrabber(TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[0], TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[1], TOP_GRABBER);

        }else if (grabber == BOTTOM_GRABBER){
            setGrabber(BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[0], BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[1], BOTTOM_GRABBER);
        }
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
                firstGlyphLift.setPower(0.3);
                //LinearOpMode.class.wait allows sensor to move away from the magnet
                linearOpMode.sleep(200);
                setLiftDirection(FIRST_LIFT, DOWN);
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
               //LinearOpMode.class.wait allows sensor to move away from the magnet
                linearOpMode.sleep(400);
                setLiftDirection(FIRST_LIFT, UP);
            }
            else{
                //Don't allow to move down any further
                firstGlyphLift.setPower(0);
            }
        }
        else{
            firstGlyphLift.setPower(scaleInput(gamepad2.left_stick_y));
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
                secondGlyphLift.setPower(-0.3);
                linearOpMode.sleep(500);
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
                linearOpMode.sleep(500);
                setLiftDirection(SECOND_LIFT, UP);
            }
            else{
                secondGlyphLift.setPower(0);
            }
        }
        else{

            secondGlyphLift.setPower(gamepad2.right_stick_y/-2);
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
        jewelArm.setPosition(0);
        linearOpMode.sleep(600);
    }

    static public void raiseJewelArms(LinearOpMode linearOpMode){
        jewelArm.setPosition(0.9);
        linearOpMode.sleep(1000);
        jewelKnocker.setPosition(0);
        linearOpMode.sleep(500);
    }

    static public void knockJewel(JDColor jewelColor, JDColor stoneColor, LinearOpMode linearOpMode){
        if(jewelColor == JDColor.NONE) {
            //do nothing
        }
        else if (jewelColor == stoneColor){
            kickOpposite(linearOpMode);
        }
        else if (jewelColor != stoneColor){
            kickSame(linearOpMode);
        }
    }


    static public void kickOpposite(LinearOpMode linearOpMode){
        jewelKnocker.setPosition(0);
        linearOpMode.sleep(1500);
    }

    static public void kickSame(LinearOpMode linearOpMode){
        jewelKnocker.setPosition(1);
        linearOpMode.sleep(1500);
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

    static public JDColor detectJewelColor(LinearOpMode linearOpMode){

        JDColor jewelColorFound = JDColor.NONE;

        long startTime = System.nanoTime();
        long elapsedTime = 0;
        float hue = 0F;

        //read color for about 2 seconds
        while( jewelColorFound == JDColor.RED || jewelColorFound == JDColor.BLUE
                || elapsedTime < 2000) {

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

            linearOpMode.telemetry.addData("hue", hsvValues[0]);
            linearOpMode.telemetry.addData("S", hsvValues[1]);
            linearOpMode.telemetry.addData("V", hsvValues[2]);
            linearOpMode.telemetry.update();

            hue  = hsvValues[0];

            if (hue >= 190 && hue <= 235) {
                jewelColorFound = JDColor.BLUE;
            }
            else if (hue <= 15 || hue >= 350) {
                jewelColorFound = JDColor.RED;
            }
            else if (hue == 0) {
                jewelColorFound = JDColor.NONE;
            }

            elapsedTime = TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime);
        }

        linearOpMode.telemetry.addData("hue", Float.toString(hue));
        linearOpMode.telemetry.addData("Jewel Color", jewelColorFound.toString());
        linearOpMode.telemetry.update();
        jewelColorSensor.enableLed(false);
        return jewelColorFound;
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
        //timer based fail safe logic
        long startTime = System.nanoTime();
        long elapsedTime = 0;

        //try to read the vumark until we find a valid vumark or for 3 seconds
        while(vuMark == RelicRecoveryVuMark.UNKNOWN  &&  elapsedTime <= 3000 && linearOpMode.opModeIsActive()){
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN){
                linearOpMode.telemetry.addData("Vumark Found", vuMark.toString());
                linearOpMode.telemetry.update();
                break;
            }
            elapsedTime = TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime);
            linearOpMode.telemetry.addData("Elapsed Time to find VuMark:", Long.toString(elapsedTime));
            linearOpMode.telemetry.update();
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
               stop();
               break;
            }

            linearOpMode.telemetry.addData("Distance to Wall", distanceToWall);
            linearOpMode.telemetry.addData("Distance to Crypto Box Wall", distanceToCryptoBoxWall);
            linearOpMode.telemetry.addData("Centimeters from Object", sideRangeSensor.cmUltrasonic());
            linearOpMode.telemetry.addData("Columns Passed", columnsPassed);
            linearOpMode.telemetry.update();
        }
    }

    //discard any accidental bad reading from the sensor
    //break out of the loop if unable to read good sensor data within 200ms
    static public double readAndFilterRangeSensor(LinearOpMode linearOpMode) {
        long startTime = System.nanoTime();
        long elapsedTime = 0;

        double distance =  sideRangeSensor.cmUltrasonic();
        while ( (distance == 255 || distance ==0 || elapsedTime < 200)  && linearOpMode.opModeIsActive() ) {
            sideRangeSensor.cmUltrasonic();
            elapsedTime = TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime);
        }
        return distance;
    }

    static public void moveUntilCryptoWallv2(double startDistance, RelicRecoveryVuMark vuMark, LinearOpMode linearOpMode) {
        int targetColumn;
        int cryptoWallMinVal = 5;

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
        boolean firstTime=true;

        double  distanceToCrypto = startDistance - cryptoWallMinVal;

        frontLeftDriveMotor.setPower(0.24);
        frontRightDriveMotor.setPower(-0.24);
        backLeftDriveMotor.setPower(0.24);
        backRightDriveMotor.setPower(-0.24);

        long startTime = System.nanoTime();
        long elapsedTime = 0;
        String msg="";
        double distance = readAndFilterRangeSensor(linearOpMode);

        while ( linearOpMode.opModeIsActive() ) {

            distance = readAndFilterRangeSensor(linearOpMode);

            // move robot past each crypto column
            //as soon as target column is seen break out of the loop and stop.
            //sometimes the distance is less than the minimum distance of 5, so check if less than 5 or less than 4
            while (  (distance <= distanceToCrypto || distance <= distanceToCrypto-1) && linearOpMode.opModeIsActive()) {

                //column increased only the first time when there is a change in distance
                if ( firstTime == true ){
                    columnsPassed++;
                    firstTime = false;
                }

                //target column is reached
                if (columnsPassed >= targetColumn ) {
                    break;
                }
                msg = Long.toString(elapsedTime) + ": "
                        + Double.toString(distance)
                        + " Column: " + Integer.toString(columnsPassed)
                        + " CryptoDistance: " + distanceToCrypto;
                linearOpMode.telemetry.addData("range:", msg);
                linearOpMode.telemetry.update();
                Log.d("JDRange", msg);
                elapsedTime = TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime);

                distance = readAndFilterRangeSensor(linearOpMode);
            }

            //reset the first time for the next columns
            if ( firstTime == false ) {
                firstTime = true;
            }

            //adjust the minimum distance based on the new reading as the robot might have drifted
            distanceToCrypto = distance - cryptoWallMinVal;

            msg = Long.toString(elapsedTime) + ": "
                    + Double.toString(distance)
                    + " Column: " + Integer.toString(columnsPassed)
                    + " CryptoDistance: " + distanceToCrypto;
            linearOpMode.telemetry.addData("range:", msg);
            linearOpMode.telemetry.update();
            Log.d("JDRange", msg);
            elapsedTime = TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime);

            //if target reached break out of the main while loop
            if (columnsPassed >= targetColumn ) {
                break;
            }

        }

        frontLeftDriveMotor.setPower(0);
        frontRightDriveMotor.setPower(0);
        backLeftDriveMotor.setPower(0);
        backRightDriveMotor.setPower(0);

    }

    //AVT Algorithm to filter range sensor values and return the sampled distance
    //calculate average value
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
