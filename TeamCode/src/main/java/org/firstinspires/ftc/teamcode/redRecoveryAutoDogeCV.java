package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.hardware.initHardwareMap;

/**
 * Created by dansm on 12/21/2017.
 */

@Autonomous(name="REDRecoveryAuto")
public class redRecoveryAutoDogeCV extends LinearOpMode{
    public void addTelemetry(String caption, String telemetryValue, boolean update){
        telemetry.addData(caption, telemetryValue);
        if(update){
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException{
        //Code to run after init is pressed

        boolean hardwareMapState = initHardwareMap(hardwareMap);

        //initServos();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        waitForStart();
        //Code to run after play is pressed

        VuforiaLocalizer vuforia;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
        while( vuMark == RelicRecoveryVuMark.UNKNOWN  || TimeUnit.NANOSECONDS.toSeconds(estimatedTime) <= 3 ){
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN){
                addTelemetry("Vumark Found", vuMark.toString(), true);
                break;
            }
        }

        //if more than 3 seconds and vumark is not found, default to LEFT
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            addTelemetry("Vumark Not Found, Defaulting to LEFT", vuMark.toString(), true);
            vuMark = RelicRecoveryVuMark.LEFT;
        }


    }
}
