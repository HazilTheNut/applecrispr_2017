package org.firstinspires.ftc.teamcode.appleCRISPR_2017.DemoBots;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.AtREVContinuousServo;
import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.AtREVServo;

/**
 * Created by Jared on 10-Mar-18.
 */

public class AtDemoTricycle extends AtDemoBot {

    private AtREVContinuousServo leftWheel;
    private AtREVContinuousServo rightWheel;

    public AtDemoTricycle(HardwareMap map) {
        super(map);
        leftWheel = new AtREVContinuousServo("left");
        rightWheel = new AtREVContinuousServo("right");

        leftWheel.init(hardwareMap);
        rightWheel.init(hardwareMap);

        leftWheel.setReversed(true);
    }

    @Override
    void linearDrive(float power) {
        //power = (float) (.5 + power/2);
        leftWheel.setPower(power);
        rightWheel.setPower(power);
    }

    @Override
    void pinwheelRotation(float power) {
        //power = (float) (.5 + power/2);
        leftWheel.setPower(power);
        rightWheel.setPower(-1 * power);
    }

    @Override
    public void testLeft(){
        leftWheel.setPower(1);
    }
    @Override
    public void testRight(){
        rightWheel.setPower(1);
    }

    @Override
    public void stop() {
        leftWheel.stop();
        rightWheel.stop();
    }
}
