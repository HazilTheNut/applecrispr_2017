package org.firstinspires.ftc.teamcode.appleCRISPR_2017.DemoBots;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Jared on 10-Mar-18.
 */

public class AtDemoBot {

    HardwareMap hardwareMap;

    public AtDemoBot( HardwareMap map){
        hardwareMap = map;
    }

    /**
     * Drive demo bot forwards
     * @param power power to set wheels (range 0-100)
     * @param time time spent in seconds
     */
    public void driveForward(int power, float time){
        float motorPower = Math.max(0, Math.min(power, 100));
        linearDrive(motorPower / 100);
        try {
            Thread.sleep((long)(time / 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Drive demo bot backwards
     * @param power power to set wheels (range 0-100)
     * @param time time spent in seconds
     */
    public void driveBackward(int power, float time){
        float motorPower = Math.max(0, Math.min(power, 100));
        linearDrive(motorPower / -100);
        try {
            Thread.sleep((long)(time / 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Pinwheel turns robot counter-clockwise
     * @param power power to set wheels (range 0-100)
     * @param time time spent in seconds
     */
    public void turnLeft(int power, float time){
        float motorPower = Math.max(0, Math.min(power, 100));
        pinwheelRotation(motorPower / -100);
        try {
            Thread.sleep((long)(time / 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Pinwheel turns robot clockwise
     * @param power power to set wheels (range 0-100)
     * @param time time spent in seconds
     */
    public void turnRight(int power, float time){
        float motorPower = Math.max(0, Math.min(power, 100));
        pinwheelRotation(motorPower / -100);
        try {
            Thread.sleep((long)(time / 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
            Thread.currentThread().interrupt();
        }
    }

    /**
     * 'Linear' refers to the robot moving either forwards or backwards.
     * Positive power refers to moving forwards, negative power meaning moving backwards
     * DemoBots should override this function
     */
    void linearDrive(float power){}

    /**
     * ! NOTE ! Positive power should refer to clockwise rotations, and negative power being counter-clockwise
     * DemoBots should override this function
     */
    void pinwheelRotation(float power){}

    /**
     * Stops robot
     * DemoBots should override this function
     */
    public void stop(){}
}
