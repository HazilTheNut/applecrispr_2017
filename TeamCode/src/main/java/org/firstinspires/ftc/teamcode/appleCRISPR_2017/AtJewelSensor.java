package org.firstinspires.ftc.teamcode.appleCRISPR_2017;

import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.AtREVPixy;

/**
 * Created by Zach on 2/4/2018.
 */

public class AtJewelSensor extends AtREVPixy {
    public final static int RED = 1;
    public final static int BLUE = 2;
    public final static int LEFT = 0;
    public final static int RIGHT = 1;
    public final static int FAILURE = -1;

    public AtJewelSensor(String name)
    {
        super(name);
    }

    private boolean checkJewelsSeen()
    {
        return true;
    }

    public int jewelLR(int RB)
    {
        if(!checkJewelsSeen()) { return FAILURE; }
        return (getLargestDetectedObjX(RB) < getLargestDetectedObjX((RB+1)%2)) ? LEFT : RIGHT;
    }
    public int redJewelLR()
    {
        if(!checkJewelsSeen()) { return FAILURE; }
        return (getLargestDetectedObjX(RED) < getLargestDetectedObjX(BLUE)) ? LEFT : RIGHT;
    }
    public int blueJewelLR()
    {
        if(!checkJewelsSeen()) { return FAILURE; }
        return (getLargestDetectedObjX(BLUE) < getLargestDetectedObjX(RED)) ? LEFT : RIGHT;
    }
}
