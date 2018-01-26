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
        CONTROL SCHEME: (One Driver) Not in Use!

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


        CONTROL SCHEME: (Two Driver) In Use!

    D-1 : RS y +/-      : MOVE Forward / Backward
    D-1 : RS x +/-      : MOVE Sideways
    D-1 : Right Trigger : MOVE Clockwise Turn
    D-1 : Left Trigger  : Move Counter-clockwise Turn

    D-1 : A             : Toggle Slow Driving

    D-2 : LS y +/-      : ARM Goal U/D Axis
    D-2 : LS x +/-      : ARM Goal I/O Axis
    D-2 : RS y +/-      : ARM Power (mainly for debug / safety reasons)

    D-2 : R / L Trigger : Manual Wrist Move
    D-2 : X             : Auto-align Wrist

    BOTH : B            : ALL STOP

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
    private AtREVServo armWrist;
    private AtREVMotor armElbow;

    //Suction
    private AtREVMotor suctionMotor;

    //Programming Variables
    private boolean isSlowDriving = false;
    private boolean slowButtonPressed = false; //Intentionally lags behind reality by one loop cycle

    private double armGoalX = 7.5; //Default starting position
    private double armGoalY = 5.5;
    private final float armSegment1 = 14.5f; //In inches
    private final float armSegment2 = 13.375f; //In inches

    private double shoulderAngleOffset = 0;
    //private final double defaultElbowAngle = 0;
    private double elbowAngleOffset = 0;

    // 1120 / 360 = 3.1111111......
    private final double shoulderEncTickToDeg = 3 * 3.111111111; //Ratio is ticks / degrees. Multiplied due to gearing ratios
    private final double elbowEncTickToDeg = 2.5 * 3.111111111; //Ratio is ticks / degrees.


    //Telemetry
    private double movementDirection = 0; //In degrees
    private double shoulderGoalAngle = 0; //In degrees
    private double elbowGoalAngle = 0; //In degrees
    private double wristGoalAngle = 0; //In degrees
    private double elbowPower = 0;
    @Override

    public void init() {

        double defaultShoulderAngle = 91.3103;
        double defaultElbowAngle = 145.8516;

        driveFL = (AtREVMotor)revModule.add(new AtREVMotor("drive-fl"));
        driveFR = (AtREVMotor)revModule.add(new AtREVMotor("drive-fr"));
        driveBL = (AtREVMotor)revModule.add(new AtREVMotor("drive-bl"));
        driveBR = (AtREVMotor)revModule.add(new AtREVMotor("drive-br"));

        armShoulder = (AtREVMotor)revModule.add(new AtREVMotor("shoulder"));
        armElbow = (AtREVMotor)revModule.add(new AtREVMotor("elbow"));
        armWrist = (AtREVServo)revModule.add(new AtREVServo("wrist", calculateWristPos(defaultShoulderAngle, defaultElbowAngle), false));

        suctionMotor = (AtREVMotor)revModule.add(new AtREVMotor("suction"));

        telemetry.addData("Init successful: ", revModule.initialize(hardwareMap));
        telemetry.update();

        shoulderAngleOffset = defaultShoulderAngle - (armShoulder.getPosition() / shoulderEncTickToDeg);
        elbowAngleOffset    = defaultElbowAngle - (armElbow.getPosition() / elbowEncTickToDeg);


        driveBL.setDirection(false);
    }

    @Override
    public void loop() {
        if (gamepad1.b || gamepad2.b) { //All stop button
            revModule.allStop();
            telemetry.addData(": =X= !!! ALL STOP !!! =X=","");
        } else {
            //The various functions of the robot are divided up for your convenience

            doMovement(); //Completed

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
        if (!slowButtonPressed && gamepad1.a){
            isSlowDriving = !isSlowDriving;
            slowButtonPressed = true;
        } else if (!gamepad1.a){
            slowButtonPressed = false;
        }

        double powerScalar = isSlowDriving ? 0.40 : 1;

        //Finally, moving!
        if (gamepad1.right_trigger > 0.1){ //Clockwise turn
            driveFL.setPower(-1 * powerScalar);
            driveBL.setPower(-1 * powerScalar);
            driveFR.setPower(powerScalar);
            driveBR.setPower(powerScalar);
        } else if (gamepad1.left_trigger > 0.1){ //Counter-clockwise turn
            driveFL.setPower(powerScalar);
            driveBL.setPower(powerScalar);
            driveFR.setPower(-1 * powerScalar);
            driveBR.setPower(-1 * powerScalar);
        } else { //Drives normally if not turning
            driveFL.setPower(flPower);
            driveBR.setPower(flPower);
            driveFR.setPower(frPower);
            driveBL.setPower(frPower);
        }
    }



    private void doArmKinematics(){
        //directArmControl(); //Use if kinematics are not working correctly

        goalPosMovement(); //Receives input and moves goal around, within the bounds of the arm's workspace

        calculateArmAngles(); //Calculations verified to be correct

        moveArm(); //Shoulder working, elbow too

        //From Direct Drive method:
        /*

        if (gamepad2.left_trigger > 0.1){
            armWrist.incrementPosition(-0.02);
        } else if (gamepad2.right_trigger > 0.1){
            armWrist.incrementPosition(0.02);
        }
        if (gamepad2.x){
            double shoulderPos = (armShoulder.getPosition() / shoulderEncTickToDeg) + shoulderAngleOffset;
            double elbowPos = (armElbow.getPosition() / elbowEncTickToDeg) + elbowAngleOffset;
            wristGoalAngle = calculateWristPos(shoulderPos, elbowPos);

            armWrist.setPosition((wristGoalAngle / 180) + 0.5);
        } else if (gamepad2.y){
            armWrist.setPosition(0.5);
        }

        /**/
    }

    private void calculateArmAngles(){
        double r = Math.sqrt(Math.pow(armGoalX,2) + Math.pow(armGoalY,2));
        double baseAngle1 = Math.atan2(armGoalY,armGoalX);
        double elbowAngle = Math.PI - Math.acos( (Math.pow(r,2) - Math.pow(armSegment1,2) - Math.pow(armSegment2,2)) / (-2 * armSegment1 * armSegment2) );
        double baseAngle2 = Math.asin(armSegment2 * Math.sin(elbowAngle) / r);
        double baseTotalAngle = baseAngle1 + baseAngle2;
        //telemetry.addData("> baseTotalAngle",baseTotalAngle);
        double radToDeg = (180 / Math.PI);

        shoulderGoalAngle = baseTotalAngle * radToDeg; //Converted to degrees
        elbowGoalAngle = elbowAngle * radToDeg;

        wristGoalAngle = calculateWristPos(shoulderGoalAngle, elbowGoalAngle);

        /*
        telemetry.addData("IK Math","");
        telemetry.addData("> baseAngle1",baseAngle1);
        telemetry.addData("> elbowAngle",elbowAngle);
        telemetry.addData("> baseAngle2",baseAngle2);
        /**/
    }

    private void goalPosMovement() {

        double safetyMargin = 0.05; //We don't want to completely ruin the kinematics with one floating point rounding error
        //I/O axis
        double goalXMovement = gamepad2.left_stick_x * -0.4f;
        if (Math.abs(goalXMovement) < 0.1) goalXMovement = 0;
        armGoalX += goalXMovement; //Receive input

        if (armGoalX < 0) armGoalX = 0; //Limits on where the goal can be
        if (armGoalX > armSegment1 + armSegment2 - safetyMargin) armGoalX = armSegment1 + armSegment2;

        double maxLengthSqrd = Math.pow(armSegment1 + armSegment2,2);
        double maxX = Math.sqrt(maxLengthSqrd - Math.pow(armGoalY,2)) - safetyMargin;
        if (goalXMovement > 0 && armGoalX > maxX) armGoalY = Math.sqrt(maxLengthSqrd - Math.pow(armGoalX,2)) - safetyMargin; //Moves y position down to allow more room to move x position

        //U/D axis
        double goalYMovement = gamepad2.left_stick_y * -0.4f;
        if (Math.abs(goalYMovement) < 0.1) goalYMovement = 0;
        armGoalY += goalYMovement; //Receive input

        if (armGoalY < 0) armGoalY = 0; //Limits on where the goal can be
        if (armGoalY > armSegment1 + armSegment2 - safetyMargin) armGoalY = armSegment1 + armSegment2;

        double maxY = Math.sqrt(maxLengthSqrd - Math.pow(armGoalX,2)) - safetyMargin;
        if (goalYMovement > 0 && armGoalY > maxY) armGoalX = Math.sqrt(maxLengthSqrd - Math.pow(armGoalY,2)); //Moves x position back to allow more room to move y position
        /**/

    }

    private void moveArm(){
        //armShoulder.powerToPosition(0.4f, shoulderGoalAngle + shoulderAngleOffset, 5);
        double shoulderCurrent = (armShoulder.getPosition() / shoulderEncTickToDeg) + shoulderAngleOffset; //In degrees
        double elbowCurrent = (armElbow.getPosition() / elbowEncTickToDeg) + elbowAngleOffset; //Also in degrees
        if ((Math.abs(gamepad2.left_stick_y) < 0.1 && Math.abs(gamepad2.left_stick_x) < 0.1)) {
            armShoulder.setPower(getArmMotorPower(shoulderCurrent, shoulderGoalAngle, 1, 6, 0.35 * scaleInput(gamepad2.right_stick_y)));
            armElbow.setPower(   getArmMotorPower(elbowCurrent,    elbowGoalAngle,    1, 6, 0.35 * scaleInput(gamepad2.right_stick_y)));
        } else {
            armShoulder.stop();
        }
    }

    private double getArmMotorPower(double currentPos, double goalPos, int margin, int slowZone, double maxPower){
        double relativePos = Math.abs(currentPos - goalPos);
        if (relativePos < margin) return 0;
        int sign = 1;
        if (currentPos < goalPos) sign = -1;
        double power = maxPower;
        if (relativePos < slowZone) power = maxPower * Math.sin( (Math.PI / 2) * (relativePos / slowZone) ); //Smoothly scale power down to zero when close to margin
        return power * sign;
    }

    private void directArmControl(){
        double shoulderPower = scaleInput(gamepad2.left_stick_y) * 0.25;
        armShoulder.setPower(shoulderPower);
        armElbow.setPower(scaleInput(gamepad2.right_stick_y) * 0.25);
        if (gamepad2.left_trigger > 0.1){
            armWrist.incrementPosition(-0.01);
        } else if (gamepad2.right_trigger > 0.1){
            armWrist.incrementPosition(0.01);
        }
        if (gamepad2.x){
            double shoulderPos = (armShoulder.getPosition() / shoulderEncTickToDeg) + shoulderAngleOffset;
            double elbowPos = (armElbow.getPosition() / elbowEncTickToDeg) + elbowAngleOffset;
            wristGoalAngle = calculateWristPos(shoulderPos, elbowPos);

            armWrist.setPosition((wristGoalAngle / 180) + 0.5);
        } else if (gamepad2.y){
            armWrist.setPosition(0.5);
        }
    }

    private double calculateWristPos (double shoulderAngle, double elbowAngle){
        return 90 + shoulderAngle - elbowAngle;
    }

    private void doMiscActions() {
        //Suction
        if ((gamepad1.left_bumper || gamepad1.dpad_down) && suctionMotor.getPosition() > 0){ //Always prioritize releasing the pressure
            suctionMotor.setPower(-0.6);
        } else if ((gamepad1.right_bumper || gamepad1.dpad_up) && suctionMotor.getPosition() < 2200){
            suctionMotor.setPower(0.6);
        } else {
            suctionMotor.setPower(0);
        }
        /*
        if (gamepad1.dpad_left){
            elbow1Offset -= 0.00075;
        }
        if (gamepad1.dpad_right){
            elbow1Offset += 0.00075;
        }
        armElbow2.setPosition(elbow1Offset);
        */
    }

    private String buildGraph(){
        String graph = "";
        int graphcols = 50;
        int aspectratio = 3;
        int totalarmlength = 28;
        int graphrows = graphcols / aspectratio / 2;

        int col = (int)(graphcols*armGoalX/totalarmlength);
        int row = graphrows/2 + (int)(graphrows*armGoalY/totalarmlength);

        // Build graph
        for (int r=0; r<graphrows; r++) {
            for (int c=0; c<graphcols; c++){
                if (r==row && c==col){
                    graph += "▓";
                }
                else if (graphcols > Math.sqrt(Math.pow(graphrows/2-r,2) + Math.pow(aspectratio*(graphcols-c),2) )){
                    graph += "█";
                }
                else {
                    graph += "░";
                }
            }
            graph += "\n";
        }
        return graph;

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
        telemetry.addData("> Shoulder Pos (Geared)", armShoulder.getPosition() / shoulderEncTickToDeg);
        telemetry.addData("> Shoulder Offset", shoulderAngleOffset);
        telemetry.addData("> Shoulder Goal", shoulderGoalAngle);
        telemetry.addData("> Shoulder Combo", (armShoulder.getPosition() / shoulderEncTickToDeg) + shoulderAngleOffset);
        telemetry.addData("> Shoulder abs(margin)", Math.abs((armShoulder.getPosition() / shoulderEncTickToDeg) + shoulderAngleOffset - shoulderGoalAngle));
        telemetry.addData("> Elbow Pos", armElbow.getPosition() / elbowEncTickToDeg);
        telemetry.addData("> Elbow Offset", elbowAngleOffset);
        telemetry.addData("> Elbow Combo", (armElbow.getPosition() / elbowEncTickToDeg) + elbowAngleOffset);
        telemetry.addData("> Elbow Goal", elbowGoalAngle);
        telemetry.addData("> Goal Pos X", armGoalX);
        telemetry.addData("> Goal Pos Y", armGoalY);
        telemetry.addData("> Goal Magnitude", Math.sqrt(Math.pow(armGoalX,2) + Math.pow(armGoalY,2)));
        telemetry.addData("> Wrist Pos", armWrist.getPosition());
        telemetry.addData("> Wrist Goal", (wristGoalAngle / 180) + 0.5);
        telemetry.addData("> Elbow Power", elbowPower);
        telemetry.addData("> Shoulder Power", armShoulder.getPower());
        telemetry.addData("> Test Rendering Characters","..... ||||| ///// !!!!! :::::  ░▒▓█");
        telemetry.addData("", "\n"+buildGraph());

        telemetry.addData("OTHER","");
        telemetry.addData("> Suction Power", suctionMotor.getPower());
        telemetry.addData("> Suction Pos", suctionMotor.getPosition());
        telemetry.addData("> Elbow Encoder Pos", armElbow.getPosition());

        telemetry.update(); //The most important method to call!
    }

    private double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.06, 0.08, 0.09, 0.11, 0.13, 0.15,
                0.18, 0.22, 0.25, 0.33, 0.40, 0.52, 0.66, 0.85, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}
