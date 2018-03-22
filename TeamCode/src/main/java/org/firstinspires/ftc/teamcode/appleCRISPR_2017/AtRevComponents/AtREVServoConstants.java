package org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents;

import java.util.AbstractMap;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;

/**INCOMPLETE
 * TODO:Make this!!
 *
 * Created by Riley on 19-Mar-18.
 */

public class AtREVServoConstants {

    private static final Object[][] servoZeros =
    {
            {"Servo1",0.45f}
    };

    public static float getServoZero(String name)
    {
        for(int i=0; i<servoZeros.length ;i++)
        {
            if(((String)servoZeros[i][0]).equals(name))
            {
                return (float)servoZeros[i][1];
            }
        }
        return 0.5f;
    }
}
