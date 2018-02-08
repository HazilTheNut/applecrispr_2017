package org.firstinspires.ftc.teamcode.appleCRISPR_2017;

import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.AtREVPixy;

/**
 * Created by Zach on 2/4/2018.
 */

public class AtJewelSensor extends AtREVPixy {
    //Fields
    public final static int RED = 1;
    public final static int BLUE = 2;
    public final static int LEFT = 0;
    public final static int RIGHT = 1;
    public final static int FAILURE = -1;

    public AtJewelSensor(String name)
    {
        super(name);
    }

    //Implement later (probably check for sizes or number of objects detected)
    private boolean checkJewelsSeen()
    {
        return true;
    }

    //Given color returns the side that it is on
    public int jewelLR(int RB)
    {
        if(!checkJewelsSeen()) { return FAILURE; }
        //                                                                                  (origin is top left corner)
        //      \/X pos. of largest (color) obj.                 for other color\/        \/if smaller then left otherwise right
        return (getLargestDetectedObjX(RB) < getLargestDetectedObjX((RB+1)%2)) ? LEFT : RIGHT;
    }
    //Returns the side that red is on
    public int redJewelLR()
    {
        if(!checkJewelsSeen()) { return FAILURE; }
        return (getLargestDetectedObjX(RED) > getLargestDetectedObjX(BLUE)) ? LEFT : RIGHT;
    }
    //Returns the side that Blue is on
    public int blueJewelLR()
    {
        if(!checkJewelsSeen()) { return FAILURE; }
        return (getLargestDetectedObjX(BLUE) > getLargestDetectedObjX(RED)) ? LEFT : RIGHT;
    }
}
