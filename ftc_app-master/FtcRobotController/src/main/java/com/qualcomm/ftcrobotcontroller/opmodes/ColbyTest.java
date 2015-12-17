package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class ColbyTest extends OpMode {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor tape;
    Servo angler;
    Servo trigger;
    Servo arm;

    final static double tapemin  = 0.20;
    final static double tapemax  = 0.90;

    double armPosition;
    double triggerPosition;
    double tapePosition;
    double angleDelta = 0.01;

    /**
     * Constructor
     */
    public ColbyTest() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
        motorFL = hardwareMap.dcMotor.get("motor_fl");
        motorFR = hardwareMap.dcMotor.get("motor_fr");
        motorBL = hardwareMap.dcMotor.get("motor_bl");
        motorBR = hardwareMap.dcMotor.get("motor_br");
        tape = hardwareMap.dcMotor.get("motor_tape");
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        angler = hardwareMap.servo.get("servo_1");
        trigger = hardwareMap.servo.get("servo_2");
        arm = hardwareMap.servo.get("servo_3");

        // assign the starting position of the wrist and claw
        tapePosition = 0.2;
        triggerPosition = .5;
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

		/*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right
        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;
        float right = throttle - direction;
        float left = throttle + direction;
        float tapeThrottle = (gamepad1.right_stick_y)/2;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        // write the values to the motors
        motorFL.setPower(left);
        motorFR.setPower(right);
        motorBL.setPower(left);
        motorBR.setPower(right);
        tape.setPower(tapeThrottle);

        // update the position of the arm.
        if (gamepad1.x) {
            triggerPosition = 0;}

        if (gamepad1.y) {
           triggerPosition =.5;}

        if (gamepad1.b) {
            triggerPosition = 1;}

        if (gamepad1.left_bumper) {
            armPosition = 1;}

        if (gamepad1.right_bumper) {
            armPosition = 0;}

        if (gamepad1.dpad_up) {
            tapePosition+=angleDelta;
        }

        if(gamepad1.dpad_down) {
            tapePosition-=angleDelta;
        }

        // clip the position values so that they never exceed their allowed range.
        armPosition = Range.clip(armPosition, 0, 1);
        triggerPosition = Range.clip(triggerPosition, 0, 1);
        tapePosition = Range.clip(tapePosition,.4, .7);

        // write position values to the wrist and claw servo
        arm.setPosition(armPosition);
        trigger.setPosition(triggerPosition);
        angler.setPosition(tapePosition);



		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
        telemetry.addData("Arm", "Arm Position: " + armPosition);
        telemetry.addData("Trigger", "Trigger Position: " + triggerPosition);
        telemetry.addData("TapeAngle", "Tape Position: " + tapePosition);


    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }


    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

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
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
