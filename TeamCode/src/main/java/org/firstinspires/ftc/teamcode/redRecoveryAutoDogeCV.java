package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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
        int vuMarkNum;
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

        while(opModeIsActive()){
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != null && vuMark != RelicRecoveryVuMark.UNKNOWN){
                addTelemetry("Vumark Found", vuMark.toString(), true);

                if(vuMark == RelicRecoveryVuMark.LEFT){vuMarkNum = 0;}
                else if(vuMark == RelicRecoveryVuMark.CENTER){vuMarkNum = 1;}
                else if(vuMark == RelicRecoveryVuMark.RIGHT){vuMarkNum = 2;}
                break;
            }
        }


    }
}
