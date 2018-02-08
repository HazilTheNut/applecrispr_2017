package org.firstinspires.ftc.teamcode.appleCRISPR_2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents.*;

import java.util.ArrayList;

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


4.5
shoulder theta =
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

    //Sensors
    private AtREVDistanceSensor distanceSensor;
    private AtREVColorSensor colorSensor;
    private AtJewelSensor jewelSensor;

    //Suction
    private AtREVMotor suctionMotor;

    //Programming Variables
    private boolean isSlowDriving = false;
    private boolean slowButtonPressed = false; //Intentionally lags behind reality by one loop cycle

    private long startingTimestamp = 0;
    private double timeElapsedSinceLastLoop = 0; //CAREFUL! its in milliseconds
    private double shoulderPreviousPosition = 0; //In degrees
    private double elbowPreviousPosition = 0; //Also in degrees

    private double armGoalX = 4.5; //Default starting position
    private double armGoalY = 4.5;
    private final float armSegment1 = 14.5f; //In inches
    private final float armSegment2 = 13.375f; //In inches
    private double baseHeightFromGround = 2.4; // Measurement
    private double suctionCupHeight = 3; // Measurement.  This is about the max; the min is around 2.75

    private double shoulderAngleOffset = 0;
    //private final double defaultElbowAngle = 0;
    private double elbowAngleOffset = 0;
    private double distanceSensorXPos = 7.5;
    private double lastShoulderMotorSpeed = 0;
    private double lastElbowMotorSpeed = 0;
    private final int maxAccel = 0;

    // 1120 / 360 = 3.1111111......
    // 1680 / 360 = 4.6666666......
    private final double shoulderEncTickToDeg = 3 * 4.666666666; //Ratio is ticks / degrees. Multiplied due to gearing ratios
    private final double elbowEncTickToDeg = 2.5 * 3.111111111; //Ratio is ticks / degrees.


    //Telemetry
    private double movementDirection = 0; //In degrees
    private double shoulderGoalAngle = 0; //In degrees
    private double elbowGoalAngle = 0; //In degrees
    private double wristGoalAngle = 0; //In degrees
    private double elbowPower = 0;

    private ArrayList<Long> timeLog = new ArrayList<>();

    @Override

    public void init() {

        double defaultShoulderAngle = 111.68;
        double defaultElbowAngle = 154.78;

        driveFL = (AtREVMotor)revModule.add(new AtREVMotor("drive-fl"));
        driveFR = (AtREVMotor)revModule.add(new AtREVMotor("drive-fr"));
        driveBL = (AtREVMotor)revModule.add(new AtREVMotor("drive-bl"));
        driveBR = (AtREVMotor)revModule.add(new AtREVMotor("drive-br"));

        armShoulder = (AtREVMotor)revModule.add(new AtREVMotor("shoulder"));
        armElbow = (AtREVMotor)revModule.add(new AtREVMotor("elbow"));
        armWrist = (AtREVServo)revModule.add(new AtREVServo("wrist", calculateWristPos(defaultShoulderAngle, defaultElbowAngle), false));

        suctionMotor = (AtREVMotor)revModule.add(new AtREVMotor("suction"));

        distanceSensor = (AtREVDistanceSensor)revModule.add(new AtREVDistanceSensor("glyph_finder"));
        colorSensor = (AtREVColorSensor)revModule.add(new AtREVColorSensor("color_sensor"));
        jewelSensor = (AtJewelSensor)revModule.add(new AtJewelSensor("jewel_sensor"));


        telemetry.addData("Init successful: ", revModule.initialize(hardwareMap));
        telemetry.update();

        shoulderAngleOffset = defaultShoulderAngle - (armShoulder.getPosition() / shoulderEncTickToDeg);
        elbowAngleOffset    = defaultElbowAngle - (armElbow.getPosition() / elbowEncTickToDeg);

        //armElbow.initEncMode();
        //armShoulder.initEncMode();

        driveBL.setDirection(false);

        //armElbow.setDirection(false);
        //armShoulder.setDirection(false);

        startingTimestamp = System.currentTimeMillis();
        shoulderPreviousPosition = armShoulder.getPosition();
        elbowPreviousPosition = armElbow.getPosition();

        timeLog.add(1L);
        timeLog.add(0L);
    }

    //private int ⊙△⊙ = 4;

    @Override
    public void loop() {
        timeElapsedSinceLastLoop = timeLog.get(1) - timeLog.get(0);

        if (gamepad1.b || gamepad2.b) { //All stop button
            revModule.allStop();
            telemetry.addData(": =X= !!! ALL STOP !!! =X=\n ಠ_ಠ  \n(((φ(◎ロ◎;)φ)))\n((((爾△爾))))\n"+
                    "ε=ε=(っ*´□`)っ  \n    =͟͟͞͞ =͟͟͞͞ ﾍ( ´Д`)ﾉ","");
        } else {
            //The various functions of the robot are divided up for your convenience

            doMovement(); //Completed

            doArmKinematics(); //Incomplete

            doMiscActions(); //Completed?
        }

        updateTelemetry(); //Always in progress

        shoulderPreviousPosition = armShoulder.getPosition();
        elbowPreviousPosition = armElbow.getPosition();

        timeLog.add(System.currentTimeMillis());
        timeLog.remove(0); //removing at index of 0 - the bottom of the stack
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

        double powerScalar = isSlowDriving ? 0.30 : 1;

        //Finally, moving!
        if (gamepad1.right_trigger > 0.1){ //Clockwise turn
            driveFL.setPower(-1 * gamepad1.right_trigger * powerScalar);
            driveBL.setPower(-1 * gamepad1.right_trigger * powerScalar);
            driveFR.setPower(gamepad1.right_trigger * powerScalar);
            driveBR.setPower(gamepad1.right_trigger * powerScalar);
        } else if (gamepad1.left_trigger > 0.1){ //Counter-clockwise turn
            driveFL.setPower(gamepad1.left_trigger * powerScalar);
            driveBL.setPower(gamepad1.left_trigger * powerScalar);
            driveFR.setPower(-1 * gamepad1.left_trigger * powerScalar);
            driveBR.setPower(-1 * gamepad1.left_trigger * powerScalar);
        } else { //Drives normally if not turning
            driveFL.setPower(flPower * powerScalar);
            driveBR.setPower(flPower * powerScalar);
            driveFR.setPower(frPower * powerScalar);
            driveBL.setPower(frPower * powerScalar);
        }
    }

    private class PositionLogEntry {
        int pos;
        long timestamp;
        PositionLogEntry(int encPos, long time) {pos = encPos; timestamp = time; }
    }

    private ArrayList<PositionLogEntry> elbowPosLog = new ArrayList<>();
    private boolean elbowPreviouslyPowered = false;
    private long elbowPowerStartTime = 0;
    private long elbowPowerEndTime = 0;

    private void logElbowEncPos(){
        if (armElbow.getPower() != 0) {
            if (!elbowPreviouslyPowered){
                elbowPreviouslyPowered = true;
                elbowPowerStartTime = System.currentTimeMillis();
                elbowPosLog.clear();
            }
            PositionLogEntry entry = new PositionLogEntry(armElbow.getPosition(), System.currentTimeMillis() - elbowPowerStartTime);
            elbowPosLog.add(entry);
            if (elbowPosLog.size() > 15) elbowPosLog.remove(0);
        } else if (elbowPreviouslyPowered){
            elbowPreviouslyPowered = false;
            elbowPowerEndTime = System.currentTimeMillis();
        }
    }

    private void doArmKinematics(){
        //directArmControl(); //Use if kinematics are not working correctly

        goalPosMovement(); //Receives input and moves goal around, within the bounds of the arm's workspace

        calculateArmAngles(); //Calculations verified to be correct

        moveArm(); //Shoulder working, elbow too

        operateWrist(); //Run wrist methods
    }

    /**
     * @return In degrees
     */
    private double getShoulderPos() { return (armShoulder.getPosition() / shoulderEncTickToDeg) + shoulderAngleOffset; }

    /**
     * "Actual" meaning the literal readout from the encoders
     * @return In degrees
     */
    private double getElbowActualPos() { return (armElbow.getPosition() / elbowEncTickToDeg) + elbowAngleOffset; }

    private void operateWrist(){
        if (gamepad2.left_trigger > 0.1){
            armWrist.incrementPosition(-0.025);
        } else if (gamepad2.right_trigger > 0.1){
            armWrist.incrementPosition(0.025);
        }
        if (gamepad2.x){
            double shoulderPos = getShoulderPos();
            double elbowPos = getElbowActualPos();
            wristGoalAngle = calculateWristPos(shoulderPos, elbowPos);

            armWrist.setPosition((wristGoalAngle / 180) + 0.5);
        } else if (gamepad2.y){
            armWrist.setPosition(0.5);
        }

    }

    /**
     * Does a lot of math that is kind of the "secret recipe" of this software. I couldn't make comments that concisely explain what exactly is going on.
     */
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
        double goalMovementScalar = -1f;
        /*
        if (gamepad2.a){
            double distance = distanceSensor.getLight(); //Get distance
            if (distance != DistanceSensor.distanceOutOfRange){ //Checks if detection error
                armGoalY = 6; //Sets y position (1 Glyph stack)
                armGoalX = distance + distanceSensorXPos;
                if (Math.sqrt(Math.pow(armGoalX,2) + Math.pow(armGoalY,2)) > armSegment1 + armSegment2 - safetyMargin){
                    armGoalX = Math.sqrt(Math.pow(armSegment1 + armSegment2 - safetyMargin, 2) - Math.pow(armGoalY,2));
                }
            }
        }
        /**/

        //I/O axis
        double goalXMovement = gamepad2.left_stick_x * goalMovementScalar;
        if (Math.abs(goalXMovement) < 0.1) goalXMovement = 0;
        armGoalX += goalXMovement; //Receive input

        if (armGoalX < 0) armGoalX = 0; //Limits on where the goal can be
        armGoalX = Math.min(armGoalX, armSegment1 + armSegment2 - 1);
        if (armGoalX > armSegment1 + armSegment2 - safetyMargin) armGoalX = armSegment1 + armSegment2;

        double maxLengthSqrd = Math.pow(armSegment1 + armSegment2,2);
        double maxX = Math.sqrt(maxLengthSqrd - Math.pow(armGoalY,2)) - safetyMargin;
        if (goalXMovement > 0 && armGoalX > maxX) armGoalY = Math.sqrt(maxLengthSqrd - Math.pow(armGoalX,2)) - safetyMargin; //Moves y position down to allow more room to move x position

        //U/D axis
        double goalYMovement = 0;
        /*/ Buttons to set height.  To be implemented later.
        if (gamepad2.x){
            armGoalY = 6 - baseHeightFromGround + suctionCupHeight;
        }
        else if (gamepad2.y){
            armGoalY = 6*2 - baseHeightFromGround + suctionCupHeight;
        }
        else if (gamepad2.b){
            armGoalY = 6*3 - baseHeightFromGround + suctionCupHeight;
        } else {
        */
        goalYMovement = gamepad2.left_stick_y * goalMovementScalar;
        if (Math.abs(goalYMovement) < 0.1) goalYMovement = 0;
        armGoalY += goalYMovement; //Receive input
        //}

        if (armGoalY < -2) armGoalY = -2; //Limits on where the goal can be
        if (armGoalY > armSegment1 + armSegment2 - safetyMargin) armGoalY = armSegment1 + armSegment2;

        double maxY = Math.sqrt(maxLengthSqrd - Math.pow(armGoalX,2)) - safetyMargin;
        if (goalYMovement > 0 && armGoalY > maxY) armGoalX = Math.sqrt(maxLengthSqrd - Math.pow(armGoalY,2)); //Moves x position back to allow more room to move y position
        /**/

    }

    private void moveArm(){
        //armShoulder.powerToPosition(0.4f, shoulderGoalAngle + shoulderAngleOffset, 5);
        double elbowCurrent = getElbowActualPos(); //Also in degrees
        if (gamepad2.dpad_up){ //Increases enc. pos
            //armElbow.setPower(0.21);
            regulatedMotorSpeed(armElbow, elbowPreviousPosition, 70);
            telemetry.addData("GamePad 2 D-UP","");
        } else if (gamepad2.dpad_down){ //Decreases enc. pos
            //armElbow.setPower(-0.21);
            regulatedMotorSpeed(armElbow, elbowPreviousPosition, -70);
            telemetry.addData("GamePad 2 D-DOWN","");
        } else if (gamepad2.dpad_left){ //Increases enc. pos
            //armShoulder.setPower(0.2);
            regulatedMotorSpeed(armShoulder, shoulderPreviousPosition, 80);
            telemetry.addData("GamePad 2 D-LEFT","");
        } else if (gamepad2.dpad_right){ //Decreases enc. pos
            //armShoulder.setPower(-0.2);
            regulatedMotorSpeed(armShoulder, shoulderPreviousPosition, -80);
            telemetry.addData("GamePad 2 D-RIGHT","");
        /**/
            //armShoulder.PIDpower((0.225 * scaleInput(gamepad2.right_stick_y)), (int)(shoulderGoalAngle * shoulderEncTickToDeg));
            //armElbow.PIDpower((0.225 * scaleInput(gamepad2.right_stick_y)), (int)(elbowMathToActual(shoulderGoalAngle, elbowGoalAngle) * elbowEncTickToDeg));
        /**/
        } else if (gamepad2.right_stick_y > -0.5) {
        // Calculate the desired encoder value
        armShoulder.setPower(getArmMotorSpeed(getShoulderPos(),       shoulderGoalAngle,                                       1, 10 , 0.20 + (0.1 * scaleInput(gamepad2.right_stick_y))));
        armElbow.setPower(   getArmMotorSpeed(getElbowActualPos(),    elbowMathToActual(shoulderGoalAngle, elbowGoalAngle),    1, 10,  0.20 + (0.1 * scaleInput(gamepad2.right_stick_y))));
        }
        /*
        } else {
            armShoulder.stop();
            armElbow.stop();
        }
        /**/
    }

    private void regulatedMotorSpeed(AtREVMotor motor, double prevPos, double goalSpeed){
        double speed = 1000 * (motor.getPosition() - prevPos) / timeElapsedSinceLastLoop; //In motor ticks / second
        telemetry.addData("> Regulated Drive", "Measured " + (int)speed + " Goal " + (int)goalSpeed);
        telemetry.addData("> Regulated Drive", "Elapsed: " + timeElapsedSinceLastLoop);
        double speedDifference = goalSpeed - speed;
        double adjustmentAmount = (0.01 * Math.abs(speedDifference / goalSpeed));
        if (Math.abs(speed) < Math.abs(goalSpeed)) adjustmentAmount = Math.max(adjustmentAmount, 0.01); //Ensures no motor lockup
        if (speed < goalSpeed){
            motor.setPower(motor.getPower() + adjustmentAmount);
        }
        if (speed > goalSpeed){
            motor.setPower(motor.getPower() - adjustmentAmount);
        }
    }

    private double getArmMotorSpeed(double currentPos, double goalPos, int margin, int slowZone, double maxPower){
        double relativePos = Math.abs(currentPos - goalPos);
        if (relativePos < margin) return 0;
        int sign = 1;
        if (currentPos > goalPos) sign = -1;
        double power = maxPower;
        if (relativePos < slowZone) power *= (relativePos / slowZone); //Smoothly scale power down to zero when close to margin
        double powerMinimum = 0.125;
        if (power < powerMinimum) power = powerMinimum;
        return power * sign;
    }

    private void directArmControl(){
        double shoulderPower = scaleInput(gamepad2.left_stick_y) * 0.25;
        armShoulder.setPower(shoulderPower);
        armElbow.setPower(scaleInput(gamepad2.right_stick_y) * 0.25);
    }

    private double calculateWristPos (double shoulderAngle, double elbowAngle){
        return 90 + shoulderAngle - elbowActualToMath(shoulderAngle, elbowAngle);
    }

    /**
     * Converts mathematical elbow angle to the actual elbow angle
     *
     * This results from the elbow not being fixed to the end of segment 1 and actually being in relation to the floor
     * @param shoulderAngle angle of the shoulder
     * @param elbowMathAngle angle of the mathematical elbow angle
     * @return the actual elbow angle
     */
    private double elbowMathToActual (double shoulderAngle, double elbowMathAngle){ return elbowMathAngle + (90 - shoulderAngle); }

    private double elbowActualToMath(double shoulderAngle, double elbowActualAngle) { return elbowActualAngle - (90 - shoulderAngle); }

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
        int graphcols = 36; //
        float aspectratio = 1.0f; //   Height / Width
        float totalarmlength = armSegment1 + armSegment2;
        int graphrows = (int) (graphcols / aspectratio / 2);

        // For placing a character where the arm wants to go
        int goalCol = (int)((graphcols-1) * (armGoalX+totalarmlength) / (totalarmlength*2))+1;
        int goalRow = graphrows - (int)((graphrows-1) * armGoalY/totalarmlength);

        // For placing a character where the arm thinks it is
        // NOT FINISHED
        int measuredCol = (int)((graphcols-1) * (armGoalX+totalarmlength) / (totalarmlength*2))+1;
        int measuredRow = graphrows - (int)((graphrows-1) * armGoalY/totalarmlength);

        int fill = Math.round((16*suctionMotor.getPosition())/2200)<=16 ? Math.round((16*suctionMotor.getPosition())/2200) : 16;
        int fillBottom = fill<8 ? fill : 8;
        int fillTop = fill>8 ? fill-8 : 0;

        String[] barArray = {" ","▁","▂","▃","▄","▅","▆","▇","█"};

        // Build graph
        for (int r=1; r<=graphrows; r++) {
            for (int c=1; c<=graphcols; c++){
                if (r==goalRow && c==goalCol){
                    graph += "▓";
                }
                else if(c==34 && r==1)
                {
                    graph += "▔";
                }
                else if(c==33 && (r==2 || r==3))
                {
                    graph += "▏";
                }
                else if(c==35 && (r==2 || r==3))
                {
                    graph += "▕";
                }
                else if(c==34 && r==2)
                {
                    graph += barArray[fillTop];
                }
                else if(c==34 && r==3)
                {
                    graph += barArray[fillBottom];
                }
                else if(c==34 && r==4)
                {
                    graph += "▄";
                }
                else if (graphcols/2 < Math.sqrt(Math.pow(aspectratio*(graphrows-r),2) + Math.pow((graphcols/2-c),2) )){
                    graph += "█";
                }
                else if ((r==(int)((6-3.25)/(totalarmlength/graphrows))) || (r==(int)((6*2-3.25)/(totalarmlength/graphrows))) ||
                        (r==(int)((6*3-3.25)/(totalarmlength/graphrows))) || (r==(int)((6*4-3.25)/(totalarmlength/graphrows)))) {
                    graph += "═";
                }
                else {
                    graph += "▒";
                }
            }
            graph += "\n";
        }
        return graph;

    }

    private void updateTelemetry(){
        // Note that the ░ character shows up as a space.  Denser drawing characters show up fine.

        telemetry.addData("MOVEMENT","");
        telemetry.addData("> Direction", movementDirection);
        telemetry.addData("> FL Power", driveFL.getPower());
        telemetry.addData("> BL Power", driveFR.getPower());
        telemetry.addData("> FR Power", driveBL.getPower());
        telemetry.addData("> BL Power", driveBR.getPower());
        telemetry.addData("> Slow Mode", isSlowDriving);

        telemetry.addData("ARM","");
        telemetry.addData("> Shoulder Offset", shoulderAngleOffset);
        telemetry.addData("> Shoulder Goal", shoulderGoalAngle);
        telemetry.addData("> Shoulder Combo", getShoulderPos());
        telemetry.addData("> Shoulder abs(margin)", Math.abs((armShoulder.getPosition() / shoulderEncTickToDeg) + shoulderAngleOffset - shoulderGoalAngle));
        telemetry.addData("> Elbow Pos", armElbow.getPosition() / elbowEncTickToDeg);
        telemetry.addData("> Elbow Offset", elbowAngleOffset);
        telemetry.addData("> Elbow Combo", getElbowActualPos());
        telemetry.addData("> Elbow Goal (Math)", elbowGoalAngle);
        telemetry.addData("> Elbow Goal (Actual)", elbowMathToActual(shoulderGoalAngle, elbowGoalAngle));
        telemetry.addData("> Goal Pos X", armGoalX);
        telemetry.addData("> Goal Pos Y", armGoalY);
        telemetry.addData("> Goal Magnitude", Math.sqrt(Math.pow(armGoalX,2) + Math.pow(armGoalY,2)));
        telemetry.addData("> Wrist Pos", armWrist.getPosition());
        telemetry.addData("> Wrist Goal", (wristGoalAngle / 180) + 0.5);
        telemetry.addData("> Elbow Power", armElbow.getPower());
        telemetry.addData("> Shoulder Power", armShoulder.getPower());

        telemetry.addData("OTHER","");
        telemetry.addData("> Suction Power", suctionMotor.getPower());
        telemetry.addData("> Suction Pos", suctionMotor.getPosition());
        telemetry.addData("> Elbow Encoder Pos", armElbow.getPosition());
        telemetry.addData("> Shoulder Encoder Pos", armShoulder.getPosition());
        telemetry.addData("> Color RED", colorSensor.getRed());
        telemetry.addData("> Color BLUE", colorSensor.getBlue());
        telemetry.addData("> Raw Voltage Distance: ", distanceSensor.reportRawVoltage());

        telemetry.addData("GRAPH","");
        telemetry.addData("", "\n"+buildGraph());

        telemetry.addData("> Time since last loop", timeElapsedSinceLastLoop);
        telemetry.addData("> Delta Shoulder Pos", armShoulder.getPosition() - shoulderPreviousPosition);
        telemetry.addData("> Shoulder ticks / second", 1000 * (armShoulder.getPosition() - shoulderPreviousPosition) / timeElapsedSinceLastLoop);
        telemetry.addData("> Delta Elbow Pos", armElbow.getPosition() - elbowPreviousPosition);
        telemetry.addData("> Elbow ticks / second", 1000 * (armElbow.getPosition() - elbowPreviousPosition) / timeElapsedSinceLastLoop);
        telemetry.addData("> Time of elbow power", elbowPowerEndTime - elbowPowerStartTime);
        telemetry.addData("","");

        telemetry.addData("Blue side", jewelSensor.blueJewelLR()==AtJewelSensor.LEFT ? "left" : "right");
        telemetry.addData("Red side", jewelSensor.redJewelLR()==AtJewelSensor.LEFT ? "left" : "right");

        logElbowEncPos();

        long timestampDiffTotal = 0;
        for (int ii = 0; ii < elbowPosLog.size(); ii++){
            if (ii == 0)
                telemetry.addData("> Elbow Log[" + ii + "]", elbowPosLog.get(ii).pos + " |:| " + elbowPosLog.get(ii).timestamp);
            else {
                telemetry.addData("> Elbow Log[" + ii + "]", elbowPosLog.get(ii).pos + " +[" + (elbowPosLog.get(ii).pos - elbowPosLog.get(ii - 1).pos) + "] |:| " + elbowPosLog.get(ii).timestamp);
                timestampDiffTotal += elbowPosLog.get(ii).timestamp - elbowPosLog.get(ii-1).timestamp;
            }
        }
        telemetry.addData("> Timestamp Avg. Diff", timestampDiffTotal / (elbowPosLog.size() + 1));

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
