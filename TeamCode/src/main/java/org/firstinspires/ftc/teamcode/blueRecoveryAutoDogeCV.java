package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.functions.initServos;
import static org.firstinspires.ftc.teamcode.hardware.initHardwareMap;


/**
 * Created by dansm on 12/21/2017.
 */

@Autonomous(name="BLUERecoveryAuto")
public class blueRecoveryAutoDogeCV extends LinearOpMode{

    public void addTelemetry(String caption, String telemetryValue, boolean update){
        telemetry.addData(caption, telemetryValue);
        if(update){
            telemetry.update();
        }
    }

    public void runOpMode() throws InterruptedException{
        //Code to run after init is pressed
        boolean hardwareMapState = initHardwareMap(hardwareMap);
        initServos();

        waitForStart();
        //Code to run after play is pressed


    }
}
