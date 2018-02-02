package org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Jared on 27-Jan-18.
 */

public class AtREVDistanceSensor extends AtREVComponent {

    private DistanceSensor sensor;

    public AtREVDistanceSensor(String componentName) {
        name = componentName;
    }

    @Override
    public boolean init(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(DistanceSensor.class, name);
        return (sensor != null);
    }

    public double getDistanceInches() {return sensor.getDistance(DistanceUnit.INCH); }

    public double getDistanceCM() { return sensor.getDistance(DistanceUnit.CM); }
}
