package org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Jared on 27-Jan-18.
 */

public class AtREVColorSensor extends AtREVComponent {

    private ColorSensor sensor;

    public AtREVColorSensor(String componentName) {
        name = componentName;
    }

    @Override
    public boolean init(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(ColorSensor.class, name);
        return (sensor != null);
    }

    public double getRed() {return sensor.red(); }

    public double getGreen() {return sensor.green(); }

    public double getBlue() {return sensor.blue(); }
}
