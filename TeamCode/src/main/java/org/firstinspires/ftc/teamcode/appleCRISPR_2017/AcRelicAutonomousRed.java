package org.firstinspires.ftc.teamcode.appleCRISPR_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.AtREVModule;
import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.AtREVMotor;
import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.AtREVPixy;
import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.AtREVServo;

/**
 * Created by Zach on 2/6/2018.
 */

@Autonomous(name = "Stromboli Auton", group = "Autonomous")
public class AcRelicAutonomousRed extends LinearOpMode {

    //The master robot control object!
    private AtREVModule revModule = new AtREVModule();

    //Drive motors, unpaired because omniwheel-based driving
    private AtREVMotor driveFL;
    private AtREVMotor driveFR;
    private AtREVMotor driveBL;
    private AtREVMotor driveBR;

    //Arm motors/servos
    private AtREVServo jewelKnocker;

    //Sensors
    private AtJewelSensor jewelSensor;

    //Constants
    private static final int turnTime = 400; // GUESS

    public void ourInit() {

        driveFL = (AtREVMotor)revModule.add(new AtREVMotor("drive-fl"));
        driveFR = (AtREVMotor)revModule.add(new AtREVMotor("drive-fr"));
        driveBL = (AtREVMotor)revModule.add(new AtREVMotor("drive-bl"));
        driveBR = (AtREVMotor)revModule.add(new AtREVMotor("drive-br"));

        jewelKnocker = (AtREVServo)revModule.add(new AtREVServo("jewelKnocker"));
        jewelSensor = (AtJewelSensor)revModule.add(new AtJewelSensor("jewelSensor"));

        telemetry.addData("Init successful: ", revModule.initialize(hardwareMap));
        telemetry.update();

        driveBL.setDirection(false);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Init
        ourInit();
        waitForStart();

        //Run

        //Jewel
        switch (jewelSensor.redJewelLR())
        {
            //Jewel not identified; skip jewel
            case AtJewelSensor.FAILURE: break;
            //Red jewel is on left; lower jewel knocker and turn left
            case AtJewelSensor.LEFT:
                jewelKnocker.setPosition(1);
                driveFL.setPower(-1);
                driveBL.setPower(-1);
                driveBR.setPower(1);
                driveFR.setPower(1);
                sleep(turnTime);
            //Red Jewel is on left; lower jewel knocker and turn left
            case AtJewelSensor.RIGHT:
                jewelKnocker.setPosition(1);
                driveFL.setPower(1);
                driveBL.setPower(1);
                driveBR.setPower(-1);
                driveFR.setPower(-1);
                sleep(turnTime);
            //Raise jewel knocker and reset position for parking step
            default:
                jewelKnocker.setPosition(0);
                driveFL.setPower(-1*driveFL.getPower());
                driveBL.setPower(-1*driveBL.getPower());
                driveBR.setPower(-1*driveBR.getPower());
                driveFR.setPower(-1*driveFR.getPower());
                sleep(turnTime);
                driveFL.setPower(0);
                driveBL.setPower(0);
                driveBR.setPower(0);
                driveFR.setPower(0);
                sleep(100);
        }

        //Parking (driving forward because we are on red side) GUESS
        driveFL.setPower(1);
        driveBL.setPower(1);
        driveBR.setPower(1);
        driveFR.setPower(1);

        sleep(1500);

        revModule.allStop();
        stop();
    }
}
