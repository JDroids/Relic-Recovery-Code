package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;

import static org.firstinspires.ftc.teamcode.functions.*;


/**
 * Created by dansm on 12/17/2017.
 */

@TeleOp(name="DogeCV Red Crypto Detection")

public class detectCryptobox extends LinearOpMode{
    @Override

    public void runOpMode() throws InterruptedException{
        //Code to run after init is pressed
        ElapsedTime runtime = new ElapsedTime();
        CryptoboxDetector cryptoboxDetector = null;
        cryptoboxDetector = new CryptoboxDetector();

        initDogeCV(hardwareMap);

        waitForStart();
        //Code to run after play is pressed
        runtime.reset();

        while(opModeIsActive()){
            telemetry.addData("isCryptoBoxDetected", cryptoboxDetector.isCryptoBoxDetected());
            telemetry.addData("isColumnDetected ",  cryptoboxDetector.isColumnDetected());

            telemetry.addData("Column Left ",  cryptoboxDetector.getCryptoBoxLeftPosition());
            telemetry.addData("Column Center ",  cryptoboxDetector.getCryptoBoxCenterPosition());
            telemetry.addData("Column Right ",  cryptoboxDetector.getCryptoBoxRightPosition());

            telemetry.addData("Elapsed Time", runtime.time());

            telemetry.update();
            if(!opModeIsActive()){
                cryptoboxDetector.disable();
            }
        }
        Thread.sleep(1000);
    }
}
