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

@Autonomous(name = "Stromboli AutonRed", group = "Autonomous")
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

        jewelKnocker = (AtREVServo)revModule.add(new AtREVServo("jewel_knocker"));
        jewelSensor = (AtJewelSensor)revModule.add(new AtJewelSensor("jewel_sensor"));

        telemetry.addData("Init successful: ", revModule.initialize(hardwareMap));
        telemetry.update();

        driveBL.setDirection(false);
    }

    private int unturnValue = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init
        ourInit();
        waitForStart();

        //Run

        //Jewel
        double turnSpeed = 0.75;
        switch (jewelSensor.redJewelLR())
        {
            //Jewel not identified; skip jewel
            case AtJewelSensor.FAILURE: break;
            //Blue jewel is on left; lower jewel knocker and turn left (Pixy is up-side down)
            case AtJewelSensor.RIGHT:
                jewelKnocker.setPosition(0.425);
                sleep(1000);
                driveFL.setPower(-1 * turnSpeed);
                driveBL.setPower(-1 * turnSpeed);
                driveBR.setPower(turnSpeed);
                driveFR.setPower(turnSpeed);
                unturnValue = 1;
                sleep(turnTime);
                break;
            //Blue Jewel is on right; lower jewel knocker and turn right (Pixy is up-side down)
            case AtJewelSensor.LEFT:
                jewelKnocker.setPosition(0.425);
                sleep(1000);
                driveFL.setPower(turnSpeed);
                driveBL.setPower(turnSpeed);
                driveBR.setPower(-1 * turnSpeed);
                driveFR.setPower(-1 * turnSpeed);
                unturnValue = -1;
                sleep(turnTime);
                break;
        }

        jewelKnocker.setPosition(0.9);
        stopMoving();
        sleep(1000);

        driveFL.setPower(turnSpeed * unturnValue);
        driveBL.setPower(turnSpeed * unturnValue);
        driveBR.setPower(-1 * turnSpeed * unturnValue);
        driveFR.setPower(-1 * turnSpeed * unturnValue);

        sleep(turnTime);
        stopMoving();
        sleep(100);

        //Parking (driving backward because we are on blue side) GUESS
        driveFL.setPower(-1);
        driveBL.setPower(-1);
        driveBR.setPower(-1);
        driveFR.setPower(-1);

        sleep(1500);

        revModule.allStop();
        stop();
    }

    private void stopMoving() {
        driveFL.setPower(0);
        driveBL.setPower(0);
        driveBR.setPower(0);
        driveFR.setPower(0);

    }
}
