package org.firstinspires.ftc.teamcode.appleCRISPR_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.AtREVModule;
import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.AtREVMotor;
import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.AtREVServo;

/**
 * Created by Jared/Riley on 07-Jan-18.
 */

@Autonomous(name = "Stromboli Auton", group = "Autonomous")
public class AcRelicAutonomous extends LinearOpMode {
    //The master robot control object!
    private AtREVModule revModule = new AtREVModule();

    //Drive motors, unpaired because omniwheel-based driving
    private AtREVMotor driveFL;
    private AtREVMotor driveFR;
    private AtREVMotor driveBL;
    private AtREVMotor driveBR;

    //Arm motors/servos
    private AtREVMotor armShoulder;
    private AtREVServo armElbow1;
    private AtREVServo armElbow2;
    private AtREVServo armWrist;
    private AtREVMotor armElbowEnc;

    private double elbow1Offset = 0;

    //Suction
    private AtREVMotor suctionMotor;


    public void ourInit() {

        driveFL = (AtREVMotor)revModule.add(new AtREVMotor("drive-fl"));
        driveFR = (AtREVMotor)revModule.add(new AtREVMotor("drive-fr"));
        driveBL = (AtREVMotor)revModule.add(new AtREVMotor("drive-bl"));
        driveBR = (AtREVMotor)revModule.add(new AtREVMotor("drive-br"));

        armShoulder = (AtREVMotor)revModule.add(new AtREVMotor("shoulder"));
        armElbowEnc = (AtREVMotor)revModule.add(new AtREVMotor("elbow"));
        armWrist = (AtREVServo)revModule.add(new AtREVServo("wrist"));

        suctionMotor = (AtREVMotor)revModule.add(new AtREVMotor("suction"));

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
        driveFL.setPower(1);
        driveBL.setPower(1);
        driveBR.setPower(1);
        driveFR.setPower(1);

        sleep(1500);

        revModule.allStop();
        stop();
    }

}
