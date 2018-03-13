package org.firstinspires.ftc.teamcode.appleCRISPR_2017;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.appleCRISPR_2017.DemoBots.AtDemoTwoMotor;

/**
 * Created by Riley on 13-Apr-17.
 *
 * Make sure to tell the code what your robot name is! (line 24ish)
 * Good luck!
 */

@TeleOp(name = "Run Demo TeleOp", group = "TeleOp")
public class AtDemoTeleOp extends OpMode {

    /**
     * This bit of code executes when you click the "init" button on the phone to run things!
     * Thus, if you want something to happen, put it in between the curly brackets.
     *
     * @throws InterruptedException when the process is interrupted.  Did you expect otherwise?
     */

    AtDemoTwoMotor robot;

    @Override
    public void init() {
        robot = new AtDemoTwoMotor(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.b){
            robot.stop();
        } else if (gamepad1.right_trigger > 0.1){
            robot.turnLeft((int)(gamepad1.right_trigger * 100), 0.05f);
        } else if (gamepad1.left_trigger > 0.1){
            robot.turnRight((int)(gamepad1.left_trigger * 100), 0.05f);
        } else if (gamepad1.right_stick_y > 0.1){
            robot.driveBackward((int)(gamepad1.right_stick_y * 100), 0.05f);
        } else if (gamepad1.right_stick_y < 0.1){
            robot.driveForward((int)(gamepad1.right_stick_y * -100), 0.05f);
        }
        telemetry.addData("gamepad1.right_stick_y",gamepad1.right_stick_y);
        telemetry.update();
    }
}


/*               H E L P F U L   T I P S !
 *               (for when you get stuck)
 * Remember to end your lines with semicolons, like this;
 * Putting anything after two slashes in the code makes the robot ignore that. //like this
 * If the robot seems to skip a command - did you tell the program to pause there for a bit before continuing? using ("sleep(a number);")
 */