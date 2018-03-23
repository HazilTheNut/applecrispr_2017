package org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents;

/**
 *
 * Created by Riley on 19-Mar-18.
 */

public class AtREVServoConstants {

    private static final Object[][] servoZeros =
    {
            {"tricycleRight",0.4875f}
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
