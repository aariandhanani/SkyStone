//This is for the programming week 1 assignment. Please note that I took the demo hardware class and changed it to have the necessary sensors (besides the import which was not changed)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public Servo    servo    = null;

    public ColorSensor sensorColor;

    public DigitalChannel digitalTouch;

    public static final double MID_SERVO =  0.5 ;
    public static final double MAX_POWER = 0.7;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public DemoHardware(){ }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize stuff
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive  = hwMap.get(DcMotor.class, "right_drive");

        servo  = hwMap.get(Servo.class, "servo");

        sensorColor = hwMap.get(ColorSensor.class, "color_distance_sensor");

        digitalTouch = hwMap.get(DigitalChannel.class, "touch_sensor");

        // Motors

        // Set direction
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servos

        // Define and initialize installed servos.
        servo.setPosition(MID_SERVO);

        // Distance sensor

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);


    }
}


