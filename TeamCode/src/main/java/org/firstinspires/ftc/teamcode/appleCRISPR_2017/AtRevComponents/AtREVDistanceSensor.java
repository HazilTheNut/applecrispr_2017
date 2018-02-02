package org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Jared on 27-Jan-18.
 */

public class AtREVDistanceSensor extends AtREVComponent {

    private DistanceSensor distanceSensor;
    private OpticalDistanceSensor ods;

    public AtREVDistanceSensor(String componentName) {
        name = componentName;
    }

    @Override
    public boolean init(HardwareMap hardwareMap) {
        ods = hardwareMap.get(OpticalDistanceSensor.class, name);
        return (ods != null);
    }
    public double getLight() {return ods.getLightDetected(); }

    //public double getLight() {return ods.getDistance(DistanceUnit.INCH); }
    //public double getDistanceCM() { return ods.getDistance(DistanceUnit.CM); }
}
