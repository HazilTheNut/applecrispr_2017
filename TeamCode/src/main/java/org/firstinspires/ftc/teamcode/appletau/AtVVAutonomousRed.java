package org.firstinspires.ftc.teamcode.appletau;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 *
 * Created by Jared and Riley on 12-Nov-16.
 */

//@Autonomous(name = "VV 2 ball Autonomous (Red Side, turning)", group = "Autonomous")
public class AtVVAutonomousRed extends AtVVAutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        // Init
        ourInit();
        telemetry.update();
        // wait for the start button to be pressed
        telemetry.addData("Hi there!  Press start when you're ready.", "");
        waitForStart();

        calibrateGyro();

        frontColorSensor.enableLed(true);    // Set the LED in the beginning
        middleColorSensor.enableLed(true);    // Set the LED in the beginning
        beaconColorSensor.enableLed(true);

        frontInitialLightReading = getCurrentFrontLightReading();
        middleInitialLightReading = getCurrentMiddleLightReading();

        /*
        //TEST OF COLOR SENSOR
        int dbgt = 0;
        while (dbgt < 20 * 300){
            telemetry.addData("MIDDLE LIGHT", getCurrentMiddleLightReading());
            telemetry.addData("MIDDLE TARGET", middleInitialLightReading);
            telemetry.addData("FRONT LIGHT", getCurrentFrontLightReading());
            telemetry.addData("FRONT TARGET", frontInitialLightReading);
            telemetry.addData("TIMER", System.currentTimeMillis() / 100);
            dbgt++;
            Thread.sleep(50);
            telemetry.addData("Front Sensor Info : ", frontColorSensor.getConnectionInfo());
            telemetry.addData("Middle Sensor Info: ", middleColorSensor.getConnectionInfo());
            telemetry.addData("Beacon Sensor Info: ", beaconColorSensor.getConnectionInfo());
            telemetry.update();
        }
        */

        //get out from the wall so we can rotate

        driveAtPower(.6f);
        telemetry.addData("GRYO VAL: ", gyro.getIntegratedZValue());
        telemetry.update();
        Thread.sleep(1100);

        stopDrive();

        //Thread.sleep(300);

        // Rotate a little, like 40ish degrees, so we point toward the white line.
        setLeftPowers(.4f, .4f);
        setRightPowers(-.4f, -.4f);

        int t = 0;
        while (t < 20*MAX_TIME_TO_ROTATE && gyro.getIntegratedZValue() < TURN_ANGLE){
            telemetry.addData("GRYO VAL: ", gyro.getIntegratedZValue());
            telemetry.update();
            Thread.sleep(50); // 20 hertz
            t++;
        }
        if (t > 20*MAX_TIME_TO_ROTATE){ // Just for debug
            System.out.println("Gyro didn't reach 45 deg in time!!!");
            telemetry.addData("Uh oh: ", "Gyro didn't reach 45 deg in time!!!");
            telemetry.update();
        }
        stopDrive();
        //Thread.sleep(500);

        //Throw a ball
        throwBall();
        thrower.setPower(0);

        collector.setPower(1f);
        Thread.sleep(3500);
        throwBall();
        thrower.setPower(0);
        collector.setPower(0);


        /*
        //unturn
        t = 0;
        while (t < 20*MAX_TIME_TO_ROTATE && gyro.getIntegratedZValue() > 0){
            telemetry.addData("GRYO VAL: ", gyro.getIntegratedZValue());
            telemetry.update();
            Thread.sleep(50); // 20 hertz
            t++;
        }
        */
        /*

        // Go forward until sees the line
        driveAtPower(.7f); //Go fast initially
        Thread.sleep(1000);
        driveAtPower(.15f); //Creep Forward for maximum German precision
        t=0;
        while (t < 20*MAX_TIME_TO_LINE && !middleSeeingLine()){
            telemetry.addData("Sensor Read:", getCurrentMiddleLightReading());
            telemetry.update();
            Thread.sleep(50); // 20 hertz
            t++;
        }
        if (t > 20*MAX_TIME_TO_LINE){ // Just for debug
            telemetry.addData("Uh oh: ", "middle sensor didn't see line in time!!!");
        } else {
            telemetry.addData("STOPPED DUE TO MIDDLE SENSOR! ", "");
            telemetry.update();
        }
        stopDrive();

        // Creep backwards, cuz we consistently overshoot a bit.
        driveAtPower(-.15f);
        Thread.sleep(200);
        stopDrive();

        telemetry.addData("POST-TURN GRYO VAL: ", preStopVal);
        //telemetry.addData("FINAL GRYO VAL: ", finalTurnAngle); -- See note on sanity
        telemetry.update();

        Thread.sleep(500);

        turnTowardBeaconColor();
        //turnTowardBeaconGyro();

        // Now creep forward towards the beacon.  As we get closer, the color sensor on the pusher
        //  should read ever-increasing values.  Use this to tell how close we are.
        driveAtPower(.4f);
        t=0;
        while (t < 20*MAX_TIME_TO_APPROACH && !(getCurrentBeaconRedReading()<4 && getCurrentBeaconBlueReading()<4)){
            telemetry.addData("", "Waiting for beacon color to be big");
            telemetry.update();
            Thread.sleep(50); // 20 hertz
            t++;
        }
        stopDrive();
        telemetry.addData("", "Done approaching beacon");
        telemetry.addData("Beacon red: ", getCurrentBeaconRedReading());
        telemetry.addData("Beacon blue: ", getCurrentBeaconBlueReading());
        telemetry.addData("", "Now pushing button on beacon");
        telemetry.update();

        if (getCurrentBeaconRedReading() > getCurrentBeaconBlueReading()){
            pushLeftButton();
        } else if (getCurrentBeaconRedReading() < getCurrentBeaconBlueReading()){
            pushRightButton();
        }
        // Pause, and then back up a little.
        Thread.sleep(500);
        telemetry.addData("", "Done with beacon!");
        telemetry.update();

        driveAtPower(-.4f);
        Thread.sleep(900);
        pushNeitherButton();

        stopDrive();
        telemetry.addData("", "That's all for now!");
        telemetry.update();


        // Done.
        telemetry.addData("Done!", "");

        //  P  E  R  F  E  C  T !
        */
    }

}
