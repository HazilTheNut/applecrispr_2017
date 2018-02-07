package org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by Zach on 2/4/2018.
 */

public class AtREVPixy extends AtREVComponent {

    private I2cDevice pixy;
    private I2cDeviceSynchImpl dataQuerier;
    private I2cAddr pixyAddress = I2cAddr.create8bit(0x54);

    public final int MAX_X = 255;
    public final int MAX_Y = 199;
    public final int MAX_W = 255;
    public final int MAX_H = 200;

    @Override
    public boolean init(HardwareMap hardwareMap){
        pixy = hardwareMap.i2cDevice.get("pixy");
        dataQuerier = new I2cDeviceSynchImpl(pixy, pixyAddress, false);
        dataQuerier.engage();
        return (pixy != null);
    }

    private byte[] getLargestDetectedObj(int signature)
    {
        if(signature>0 && signature<6) { return dataQuerier.read(0x50+signature, 5); }
        else { return null;}
    }
    private byte[] getLargestDetectedObj()
    {
        return dataQuerier.read(0x50, 6);
    }

    public int getNumberDetectedObj(int signature)
    {
        return getLargestDetectedObj(signature)[0];
    }
    public int getLargestDetectedObjX(int signature)
    {
        return getLargestDetectedObj(signature)[1];
    }
    public int getLargestDetectedObjY(int signature)
    {
        return getLargestDetectedObj(signature)[2];
    }
    public int getLargestDetectedObjW(int signature)
    {
        return getLargestDetectedObj(signature)[3];
    }
    public int getLargestDetectedObjH(int signature)
    {
        return getLargestDetectedObj(signature)[4];
    }


    public int getLargestDetectedObjSig()
    {
        return ((int)getLargestDetectedObj()[0] << 8) | getLargestDetectedObj()[1];
    }
    public int getLargestDetectedObjX()
    {
        return getLargestDetectedObj()[2];
    }
    public int getLargestDetectedObjY()
    {
        return getLargestDetectedObj()[3];
    }
    public int getLargestDetectedObjW()
    {
        return getLargestDetectedObj()[4];
    }
    public int getLargestDetectedObjH()
    {
        return getLargestDetectedObj()[5];
    }

    public int[] getLargestDectedObjInfo(int signature)
    {
        byte[] objBytes = getLargestDetectedObj(signature);
        int[] objInfo = new int[5];
        for(int i=0; i<5; i++) { objInfo[i] = objBytes[i]; }
        return objInfo;
    }
    public int[] getLargestDectedObjInfo()
    {
        byte[] objBytes = getLargestDetectedObj();
        int[] objInfo = new int[5];
        objInfo[0] = ((int)objBytes[0] << 8) | objBytes[1];
        for(int i=1; i<6; i++) { objInfo[i] = objBytes[i]; }
        return objInfo;
    }

    public int[] getLargestDectedObjXY(int signature)
    {
        byte[] objBytes = getLargestDetectedObj(signature);
        int[] objInfo = new int[2];
        for(int i=1; i<3; i++) { objInfo[i] = objBytes[i]; }
        return objInfo;
    }
    public int[] getLargestDectedObjXY()
    {
        byte[] objBytes = getLargestDetectedObj();
        int[] objInfo = new int[2];
        for(int i=2; i<4; i++) { objInfo[i] = objBytes[i]; }
        return objInfo;
    }

    public int[] getLargestDectedObjWH(int signature)
    {
        byte[] objBytes = getLargestDetectedObj(signature);
        int[] objInfo = new int[2];
        for(int i=3; i<5; i++) { objInfo[i] = objBytes[i]; }
        return objInfo;
    }
    public int[] getLargestDectedObjWH()
    {
        byte[] objBytes = getLargestDetectedObj();
        int[] objInfo = new int[2];
        for(int i=4; i<6; i++) { objInfo[i] = objBytes[i]; }
        return objInfo;
    }

    public int getLargestDectedObjArea(int signature)
    {
        byte[] objBytes = getLargestDetectedObj(signature);
        int W = objBytes[3];
        int H = objBytes[4];
        return H*W;
    }
    public int getLargestDectedObjArea()
    {
        byte[] objBytes = getLargestDetectedObj();
        int W = objBytes[4];
        int H = objBytes[5];
        return H*W;
    }

    public int getLargestDectedObjWHRatio(int signature)
    {
        byte[] objBytes = getLargestDetectedObj(signature);
        int W = objBytes[3];
        int H = objBytes[4];
        return W/H;
    }
    public int getLargestDectedObjWHRatio()
    {
        byte[] objBytes = getLargestDetectedObj();
        int W = objBytes[4];
        int H = objBytes[5];
        return W/H;
    }
}
