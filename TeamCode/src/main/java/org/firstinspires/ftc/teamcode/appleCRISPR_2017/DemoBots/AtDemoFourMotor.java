package org.firstinspires.ftc.teamcode.appleCRISPR_2017.DemoBots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.AtREVMotor;
import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.AtREVMotorPair;

/**
 * Created by Riley on 10-Mar-18.
 */

public class AtDemoFourMotor extends AtDemoBot {

    AtREVMotorPair leftWheel;
    AtREVMotorPair rightWheel;

    public AtDemoFourMotor(HardwareMap map) {
        super(map);
        leftWheel = new AtREVMotorPair("drive-fl:drive-bl");
        rightWheel = new AtREVMotorPair("drive-fr:drive-br");

        leftWheel.init(hardwareMap);
        rightWheel.init(hardwareMap);
        leftWheel.setOneDirection(false);
        rightWheel.setOneDirection(false);
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
