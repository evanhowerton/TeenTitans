package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

public class RedShort extends LinearOpMode {
    DcMotor motorRightA;
    DcMotor motorRightB;
    DcMotor motorLeftA;
    DcMotor motorLeftB;
    DcMotor tapeExt;

    // this is a test for github
    Servo clawBody;
    Servo trigger;
    Servo tapeAngle;
    Servo presser;

    OpticalDistanceSensor ODS;
    TouchSensor touchSensor;
    ColorSensor colorSensor;


    @Override
    public void runOpMode() throws InterruptedException {

        motorRightA = hardwareMap.dcMotor.get("motor_fr");
        motorRightB = hardwareMap.dcMotor.get("motor_br");
        motorLeftA = hardwareMap.dcMotor.get("motor_fl");
        motorLeftB = hardwareMap.dcMotor.get("motor_bl");
        tapeExt = hardwareMap.dcMotor.get("motor_ext");
        motorLeftA.setDirection(DcMotor.Direction.REVERSE);
        motorLeftB.setDirection(DcMotor.Direction.REVERSE);

        clawBody = hardwareMap.servo.get("servo_1");
        trigger = hardwareMap.servo.get("servo_2");
        tapeAngle = hardwareMap.servo.get("servo_3");
        presser = hardwareMap.servo.get("servo_4");

        ODS = hardwareMap.opticalDistanceSensor.get("sensor_ods");
        touchSensor = hardwareMap.touchSensor.get("sensor_touch");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");



        trigger.setPosition(.5);
        clawBody.setPosition(1);


        // Wait for the start button to be pressed
        waitForStart();
        drive(0, 1, .5);
        drive(.5, 1, .5);
        turn(49, 1, .5);
        drive(6.7, 1, .5);
        turn(54, 1, .5);
        //drive(.75, 1, .5);
        colorPress();

        claw(.5);
        blueRamp();

    }

    public void drive(double dist, double pow, double pause) throws InterruptedException {

        double maxVel= 2.31;

        motorLeftA.setPower(-pow);
        motorRightA.setPower(-pow);
        motorLeftB.setPower(-pow);
        motorRightB.setPower(-pow);

        sleep((long) (1000 * (Math.abs(dist / (maxVel * pow)))));

        motorLeftA.setPower(0);
        motorRightA.setPower(0);
        motorLeftB.setPower(0);
        motorRightB.setPower(0);

        sleep((long) (pause * 1000));
    }

    public void turn(double theta, double pow, double pause) throws InterruptedException {

        double period = 3.2;

        if(theta>0){
            motorLeftA.setPower(-pow);
            motorRightA.setPower(pow);
            motorLeftB.setPower(-pow);
            motorRightB.setPower(pow);
        }

        if(theta<0){
            motorLeftA.setPower(pow);
            motorRightA.setPower(-pow);
            motorLeftB.setPower(pow);
            motorRightB.setPower(-pow);
        }

        sleep((long) (1000 * Math.abs((theta / 360) * period)));

        motorLeftA.setPower(0);
        motorRightA.setPower(0);
        motorLeftB.setPower(0);
        motorRightB.setPower(0);

        sleep((long)(pause*1000));

    }
    public void claw(double pause) throws InterruptedException {

        clawBody.setPosition(0);

        sleep(1000);

        clawBody.setPosition(1);

        sleep((long) (pause * 1000));

    }

    public void setAngle(double theta, double pause) throws InterruptedException{
        double angle = (3*theta) / 180;
        angle = Range.clip(angle, 0, 1);
        tapeAngle.setPosition(angle);

        sleep((long) (pause * 1000));

    }

    public void tapeExt(double length, double pow, double pause) throws InterruptedException {

        double maxVel=2;

        tapeExt.setPower(-pow);
        sleep((long) (1000 * (Math.abs(length / (maxVel * pow)))));

        tapeExt.setPower(0);
        sleep((long) (pause * 1000));

    }

    public void tapePull(double length, double pow, double pause) throws InterruptedException{
        double maxVel=2;

        tapeExt.setPower(-pow);
        motorLeftA.setPower(pow);
        motorRightA.setPower(pow);
        motorLeftB.setPower(pow);
        motorRightB.setPower(pow);
        sleep((long) (1000 * (Math.abs(length / (maxVel * pow)))));

        tapeExt.setPower(0);
        motorLeftA.setPower(0);
        motorRightA.setPower(0);
        motorLeftB.setPower(0);
        motorRightB.setPower(0);
        sleep((long) (pause * 1000));
    }

    public void end() throws InterruptedException {

        motorLeftA.setPowerFloat();
        motorLeftB.setPowerFloat();
        motorRightA.setPowerFloat();
        motorRightB.setPowerFloat();

    }

    public void blueRamp() throws InterruptedException {
        drive(1, -1, .5);
        turn(-50, 1, .5);
        drive(2.4, -1, .5);
        turn(92, 1, .5);
        drive(3.25, 1, .5);

        setAngle(60.0, .5);
        tapeExt(3, 1, 1.5);
        setAngle(0.0, 2);
        tapePull(4.5, -.5, .5);
    }

    public void colorPress() throws InterruptedException {
        //assumes robot goes past line
        double reflectance = ODS.getLightDetected();
        double threshhold =.25;
        Boolean touched = false;
        while(touched==false) {
            if (touchSensor.isPressed()) {
                touched=true;
            }
            else if (reflectance >= threshhold) {
                motorRightA.setPower(-.2);
                motorRightB.setPower(-.2);
                motorLeftA.setPower(0);
                motorRightB.setPower(0);
                sleep(250);
            }

            else {
                motorRightA.setPower(0);
                motorRightB.setPower(0);
                motorLeftA.setPower(-.2);
                motorRightB.setPower(-.2);
                sleep(250);
            }
            telemetry.addData("Reflectance", "Reflectance: " + reflectance);
        }
        if (colorSensor.red() > colorSensor.blue()) {
            presser.setPosition(.25);
        }
        else if(colorSensor.red() > colorSensor.blue()) {
            presser.setPosition(.75);
        }
        else{ }
    }

}