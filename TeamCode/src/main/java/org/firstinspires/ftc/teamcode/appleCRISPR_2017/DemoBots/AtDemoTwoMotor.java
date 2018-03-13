package org.firstinspires.ftc.teamcode.appleCRISPR_2017.DemoBots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.AtREVMotor;

/**
 * Created by Riley on 10-Mar-18.
 */

public class AtDemoTwoMotor extends AtDemoBot {

    AtREVMotor leftWheel;
    AtREVMotor rightWheel;

    public AtDemoTwoMotor(HardwareMap map) {
        super(map);
        leftWheel = new AtREVMotor("left");
        rightWheel = new AtREVMotor("right");

        leftWheel.init(hardwareMap);
        rightWheel.init(hardwareMap);
        rightWheel.setDirection(false);
    }

    @Override
    void linearDrive(float power) {
        leftWheel.setPower(power);
        rightWheel.setPower(power);
    }

    @Override
    void pinwheelRotation(float power) {
        leftWheel.setPower(power);
        rightWheel.setPower(-1 * power);
    }

    @Override
    public void stop() {
        leftWheel.stop();
        rightWheel.stop();
    }
}
