
package org.firstinspires.ftc.teamcode;

/**
 * Created by dansm on 12/11/2017.
 */

//A file to hold all constants

public class constants{
    static double STRAFING_LIMIT = 0.1;
    static double[] SERVO_GRABBER_INIT_POSITION = new double[]{0.5, 0.3};
    static double[] SERVO_GRABBER_CLOSE_POSITION = new double[]{0.0, 0.8};
    static double[] SERVO_GRABBER_OPEN_POSITION = new double[]{0.2, 0.6};
    static double[] SERVO_GRABBER_WIDE_OPEN_POSITION = new double[]{0.5, 0.3};

    static double JEWEL_KNOCKER_INIT_POSITION = 0;
    static double JEWEL_ARM_INIT_POSITION = 0.9;

    static int FIRST_LIFT = 1;
    static int SECOND_LIFT = 2;
    static int UP = 1;
    static int DOWN = -1;

    public enum JewelColor {NONE, RED, BLUE};

    static int BOTH_GRABBERS = 0;
    static int BOTTOM_GRABBER = 1;
    static int TOP_GRABBER = 2;

}
