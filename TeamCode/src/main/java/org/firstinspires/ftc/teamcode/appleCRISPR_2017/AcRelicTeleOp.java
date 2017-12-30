package org.firstinspires.ftc.teamcode.appleCRISPR_2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.*;

/**
 * Created by Jared on 30-Dec-17.
 */


@TeleOp(name = "Stromboli Drive", group = "TeleOp")
public class AcRelicTeleOp extends OpMode{

    /*
        CONTROL SCHEME:

    LS y +/-   :  ARM Up / Down
    LS x +/-   :  ARM Inward / Outward
    RS y +/-   :  MOVE Forward / Backward
    RS x +/-   :  MOVE Sideways

    A          :  MOVE slowly
    B          :  ALL Stop
    X          :  ARM Mirror Position
    Y          :  ARM Sensor Grab (sets cup position to sensed distance) [Feature may not be implemented]

    R Trigger  :  MOVE clockwise turn
    L Trigger  :  MOVE counter-clockwise turn
    R Bumper   :  SUCTION Grip / Grab
    L Bumper   :  SUCTION Release

    D-Pad Up   :  SUCTION Grip / Grab
    D-Pad Down :  SUCTION Release
    // Use if bumpers don't work

    */

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

    //Suction
    private AtREVMotor suctionMotor;

    //Programming Variables
    private boolean isSlowDriving = false;
    private boolean slowButtonPressed = false; //Intentionally lags behind reality by one loop cycle

    //Telemetry
    private int movementDirection = 0; //In degrees
    private int shoulderGoalAngle = 0; //In degrees
    private int elbowGoalAngle = 0; //In degrees

    @Override
    public void init() {

        driveFL = (AtREVMotor)revModule.add(new AtREVMotor("drive-fl"));
        driveFR = (AtREVMotor)revModule.add(new AtREVMotor("drive-fr"));
        driveBL = (AtREVMotor)revModule.add(new AtREVMotor("drive-bl"));
        driveBR = (AtREVMotor)revModule.add(new AtREVMotor("drive-br"));

        armShoulder = (AtREVMotor)revModule.add(new AtREVMotor("shoulder"));
        armElbow1 = (AtREVServo)revModule.add(new AtREVServo("elbow-1"));
        armElbow2 = (AtREVServo)revModule.add(new AtREVServo("elbow-2"));

        suctionMotor = (AtREVMotor)revModule.add(new AtREVMotor("suction"));

        telemetry.addData("Init successful: ", revModule.initialize(hardwareMap));
        telemetry.update();
    }

    @Override
    public void loop() {

    }

    private void doMovement(){}

    private void doArmKinematics(){}

    private void doMiscActions() {
        if (gamepad1.b){
            revModule.allStop();
            telemetry.addData("=x= ALL STOP! =x=","");
        }
    }

    private void updateTelemetry(){

        telemetry.addData("MOVEMENT","");
        telemetry.addData("> Direction       : ", movementDirection);
        telemetry.addData("> FL Power    : ", driveFL.getPower());
        telemetry.addData("> BL Power    : ", driveFR.getPower());
        telemetry.addData("> FR Power    : ", driveBL.getPower());
        telemetry.addData("> BL Power    : ", driveBR.getPower());
        telemetry.addData("> Slow Mode       : ", isSlowDriving);

        telemetry.addData("ARM","");
        telemetry.addData("> Shoulder Goal : ", shoulderGoalAngle);
        telemetry.addData("> Shoulder Pos  : ", armShoulder.getPosition() / 3.1111111); // 1120 / 360 = 3.11111111.....
        telemetry.addData("> Elbow Goal : ", elbowGoalAngle);
        telemetry.addData("> Elbow Pos  : ", "TODO");

        telemetry.addData("OTHER","");
        telemetry.addData("> Suction Power : ", suctionMotor.getPower());

        telemetry.update(); //The most important method to call!
    }
}
