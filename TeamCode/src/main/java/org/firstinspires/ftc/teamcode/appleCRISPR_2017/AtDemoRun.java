package org.firstinspires.ftc.teamcode.appleCRISPR_2017;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.appleCRISPR_2017.DemoBots.AtDemoTwoMotor;

/**
 * Created by Riley on 13-Apr-17.
 *
 * Make sure to tell the code what your robot name is! (line 24ish)
 * Good luck!
 */

@Autonomous(name = "Bunny Slope", group = "Autonomous")
public class AtDemoRun extends LinearOpMode {

    /**
     * This bit of code executes when you click the "init" button on the phone to run things!
     * Thus, if you want something to happen, put it in between the curly brackets.
     *
     * @throws InterruptedException when the process is interrupted.  Did you expect otherwise?
     */
    @Override
    public void runOpMode() throws InterruptedException {

        AtDemoTwoMotor robot = new AtDemoTwoMotor(hardwareMap);
        waitForStart();

        robot.driveForward(75, 3);
        telemetry.addData("FORWARD","");
        telemetry.update();

        robot.stop();

        robot.turnLeft(75, 3);
        telemetry.addData("CCWISE","");
        telemetry.update();

        robot.stop();
        robot.sleep(2);

        robot.turnRight(75, 3);
        telemetry.addData("C-WISE","");
        telemetry.update();

        robot.stop();

        robot.driveBackward(75, 3);
        telemetry.addData("BACKWARD","");
        telemetry.update();

        robot.stop();
    }
}


/*               H E L P F U L   T I P S !
 *               (for when you get stuck)
 * Remember to end your lines with semicolons, like this;
 * Putting anything after two slashes in the code makes the robot ignore that. //like this
 * If the robot seems to skip a command - did you tell the program to pause there for a bit before continuing? using ("sleep(a number);")
 */