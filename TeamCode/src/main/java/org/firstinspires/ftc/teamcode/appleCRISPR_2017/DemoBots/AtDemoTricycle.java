package org.firstinspires.ftc.teamcode.appleCRISPR_2017.DemoBots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.AtREVServo;

/**
 * Created by Jared on 10-Mar-18.
 */

public class AtDemoTricycle extends AtDemoBot {

    AtREVServo leftWheel;
    AtREVServo rightWheel;

    public AtDemoTricycle(HardwareMap map) {
        super(map);
        leftWheel = new AtREVServo("left");
        rightWheel = new AtREVServo("right");

        leftWheel.init(hardwareMap);
        rightWheel.init(hardwareMap);
    }

    @Override
    void linearDrive(float power) {
        power = (float) (.5 + power/2);
        leftWheel.setPosition(power);
        rightWheel.setPosition(power);
    }

    @Override
    void pinwheelRotation(float power) {
        power = (float) (.5 + power/2);
        leftWheel.setPosition(power);
        rightWheel.setPosition(-power);
    }

    @Override
    public void stop() {
        leftWheel.stop();
        rightWheel.stop();
    }
}
