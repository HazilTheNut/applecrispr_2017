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

    private double elbow1Offset = 0;

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
        armElbow1 = (AtREVServo)revModule.add(new AtREVServo("elbow-1", 0.5));
        armElbow2 = (AtREVServo)revModule.add(new AtREVServo("elbow-2", 0.475));

        suctionMotor = (AtREVMotor)revModule.add(new AtREVMotor("suction"));

        telemetry.addData("Init successful: ", revModule.initialize(hardwareMap));
        telemetry.update();

        driveBL.setDirection(false);
    }

    @Override
    public void loop() {
        if (gamepad1.b) { //All stop button
            revModule.allStop();
            telemetry.addData(": =X= !!! ALL STOP !!! =X=","");
        } else {
            //The various functions of the robot are divided up for your convenience

            doMovement(); //Completed?

            doArmKinematics(); //Incomplete

            doMiscActions(); //Completed?
        }

        updateTelemetry(); //Always in progress
    }

    private void doMovement(){
        //Omniwheel calculations
        // Maths at https://docs.google.com/spreadsheets/d/1OgPl5HwFHhcrxL53pKGzzdltQKPVZg9Mb06K4MN4qeI/edit?usp=sharing
        double x = gamepad1.right_stick_x;
        double y = gamepad1.right_stick_y;
        double joy_angle = Math.atan2(y, x);
        double rotatedAngle = joy_angle - Math.PI/4;
        double speed = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));

        rotatedAngle = Math.abs(rotatedAngle - 0)             < .09 ? 0             : rotatedAngle;  // if we're close to going a cardinal direction, snap to that.
        rotatedAngle = Math.abs(rotatedAngle - Math.PI / 2)   < .09 ? Math.PI / 2   : rotatedAngle;
        rotatedAngle = Math.abs(rotatedAngle - Math.PI)       < .09 ? Math.PI       : rotatedAngle;
        rotatedAngle = Math.abs(rotatedAngle - Math.PI * 3/2) < .09 ? Math.PI * 3/2 : rotatedAngle;

        float frPower = (float)(speed * Math.cos(rotatedAngle));
        float flPower = (float)(speed * Math.sin(rotatedAngle));

        if (frPower > .02 && frPower < -.02) frPower =  0; //Deadbanding
        if (flPower >  .02 && flPower < -.02) flPower =  0; //Clipping power

        //Setting joystick values
        double powerScalar = isSlowDriving ? 0.35 : 1;

        if (gamepad1.right_trigger > 0.1){ //Clockwise turn
            driveFL.setPower(powerScalar);
            driveBL.setPower(powerScalar);
            driveFR.setPower(-1 * powerScalar);
            driveBR.setPower(-1 * powerScalar);
        } else if (gamepad1.left_trigger > 0.1){ //Counter-clockwise turn
            driveFL.setPower(-1 * powerScalar);
            driveBL.setPower(-1 * powerScalar);
            driveFR.setPower(powerScalar);
            driveBR.setPower(powerScalar);
        } else { //Drives normally if not turning
            driveFL.setPower(flPower);
            driveBR.setPower(flPower);
            driveFR.setPower(frPower);
            driveBL.setPower(frPower);
        }
    }



    private void doArmKinematics(){}

    private void doMiscActions() {
        //Suction
        if (gamepad1.left_bumper || gamepad1.dpad_down){ //Always prioritize releasing the pressure
            suctionMotor.setPower(0.5);
        } else if (gamepad1.right_bumper || gamepad1.dpad_up){
            suctionMotor.setPower(-0.5);
        } else {
            suctionMotor.setPower(0);
        }
        if (gamepad1.dpad_left){
            elbow1Offset -= 0.00075;
        }
        if (gamepad1.dpad_right){
            elbow1Offset += 0.00075;
        }
        armElbow2.setPosition(elbow1Offset);
    }

    private void updateTelemetry(){

        telemetry.addData("MOVEMENT","");
        telemetry.addData("> Direction", movementDirection);
        telemetry.addData("> FL Power", driveFL.getPower());
        telemetry.addData("> BL Power", driveFR.getPower());
        telemetry.addData("> FR Power", driveBL.getPower());
        telemetry.addData("> BL Power", driveBR.getPower());
        telemetry.addData("> Slow Mode", isSlowDriving);

        telemetry.addData("ARM","");
        telemetry.addData("> Shoulder Goal", shoulderGoalAngle);
        telemetry.addData("> Shoulder Pos", armShoulder.getPosition() / 3.1111111); // 1120 / 360 = 3.11111111.....
        telemetry.addData("> Elbow Goal", elbowGoalAngle);
        telemetry.addData("> Elbow Pos", "TODO");
        telemetry.addData("> Elbow 1 Offset", elbow1Offset);
        telemetry.addData("> Elbow Servo 1 Pos", armElbow1.getPosition());
        telemetry.addData("> Elbow Servo 2 Pos", armElbow2.getPosition());

        telemetry.addData("OTHER","");
        telemetry.addData("> Suction Power : ", suctionMotor.getPower());

        telemetry.update(); //The most important method to call!
    }
}
