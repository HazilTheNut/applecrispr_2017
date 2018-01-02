package org.firstinspires.ftc.teamcode.appleCRISPR_2017.AtRevComponents;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Tracks both a motor and an encoder. This class assumes use of an AndyMark NeverRest motor
 * encoder. We use the 40 model for the drive train.
 * <p/>
 * The listed properties are 280 cycles per revolution and 1120 ticks per revolution. Here are the
 * full specs:
 *
 * Gearbox Reduction: 40:1
 * Voltage: 12 volt DC
 * No Load Free Speed, at gearbox output shaft: 160 rpm
 * No Load Free Speed, motor only: 6,600 rpm
 * Gearbox Output Power: 14W
 * Stall Torque: 350 oz-in
 * Stall Current: 11.5 amps
 * Force Needed to Break Gearbox: 1478 oz-in
 * Minimum torque needed to back drive: 12.8 oz-in
 * Output pulse per revolution of Output Shaft (ppr): 1120 (280 rises of Channel A)
 * Output pulse per revolution of encoder shaft (ppr): 28 (7 rises of Channel A)
 *
 * That means that 1120 encoder counts = 1 rotation = 360 degrees
 */
public class AtREVServo extends AtREVComponent {

    private Servo servo;
    private double currentPos = 1;

    private boolean isContinuousRotation = false;
    private double motionlessPosition = 1;

    public AtREVServo(String servoName) {
        name = servoName;
    }

    public AtREVServo(String servoName, double zeroMotionPos){ //If you are setting a zero-motion position, then the servo must be continuous rotation
        name = servoName;
        isContinuousRotation = true;
        motionlessPosition = zeroMotionPos;
        currentPos = motionlessPosition;
    }

    @Override
    public boolean init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, name);
        servo.setPosition(currentPos);
        return (servo != null);
    }

    public void setPosition(double position){
        servo.setPosition(position);
        currentPos = position; //Keep current pos updated
    }

    public double getPosition() { return servo.getPosition(); }

    public void incrementPosition(double inc){
        currentPos += inc;
        double MAX_POS = 1.0;
        currentPos = Math.min(MAX_POS, currentPos); //CAREFUL use min() on MAX
        double MIN_POS = 0.0;
        currentPos = Math.max(MIN_POS, currentPos); //...and max() on MIN

        setPosition(currentPos);
    }

    public double getMotionlessPosition() { return motionlessPosition; }

    @Override
    public void stop(){
        if (isContinuousRotation){
            setPosition(motionlessPosition);
        } else {
            setPosition(servo.getPosition());
        }
        currentPos = servo.getPosition();
    }
}