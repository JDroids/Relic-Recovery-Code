package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.constants.*;
import static org.firstinspires.ftc.teamcode.functions.*;
import static org.firstinspires.ftc.teamcode.hardware.*;

/**
 * Created by dansm on 12/21/2017.
 */

@Autonomous(name="REDRecoveryAuto")
public class redRecoveryAutoDogeCV extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException{

        //Code to run after init is pressed

        boolean hardwareMapState = initHardwareMap(hardwareMap);

        initServos();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imuSensor.initialize(parameters);
        int MaxValue = 255;
        double distanceToWall = sideRangeSensor.cmUltrasonic();;

        //filter bad data maximum value
        while ( !(distanceToWall < MaxValue) && !this.isStopRequested() ) {
            distanceToWall = sideRangeSensor.cmUltrasonic();
            telemetry.addData("Distance to wall", distanceToWall);
            telemetry.update();
        }

        telemetry.addData("Distance to wall", distanceToWall);
        telemetry.update();

        waitForStart();

       //Code to run after play is pressed
        telemetry.addData("Vumark:", "Initializing");
        telemetry.update();
        RelicRecoveryVuMark vuMark = getVumark(this, hardwareMap);
        telemetry.addData("Vumark:", vuMark.toString());
        telemetry.update();

        sleep(1000);

        closeGrabber(BOTTOM_GRABBER);

        moveUntilCryptoWallv2(distanceToWall,vuMark, this);

        //turn(90, this);

        //sleep(100);

        //moveForTime(0.3, 300, this);

        //openGrabber(BOTTOM_GRABBER);

        //moveForTime(-0.3, 100, this);

        //openGrabberWide(BOTTOM_GRABBER);

        //time to look for the second and third glyph

    }
}
