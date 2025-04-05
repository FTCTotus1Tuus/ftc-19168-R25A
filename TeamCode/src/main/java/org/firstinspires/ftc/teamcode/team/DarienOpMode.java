// To run on the robot:
// 1. Make Module   (Top ribbon)
// 2. Run           (Top ribbon)

package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class DarienOpMode extends LinearOpMode {

    // HARDWARE COMPONENTS
    public DcMotor omniMotor0; // left front
    public DcMotor omniMotor1; // right front
    public DcMotor omniMotor2; // left rear
    public DcMotor omniMotor3; // right rear
    //    public IMU imu;
    public GoBildaPinpointDriver odo;

    // HARDWARE FIXED CONSTANTS
    public static double encoderResolution = 537.7; //no change unless we change motors
    public double wheelDiameter = 3.75; // inches
    public double constMult = (wheelDiameter * (Math.PI));
    public double inchesToEncoder = encoderResolution / constMult;
    public static double rotationTolerance = 2; //in degrees
    public static double power = 0.3;
    public static double powerIntakeWheelToPickupSample = 0.9;
    public static double powerIntakeWheelToEjectSample = -0.25;
    public static double powerIntakeSlideIn = -0.45;
    public static double PI = 3.1416;

    // HARDWARE TUNING CONSTANTS
    public int encoderPos0, encoderPos1, encoderPos2, encoderPos3;
    public int encoderPos;
    public double regularDivBy = 2;
    public double turboDivBy = 1;
    public boolean turboBoost;

    // Servo tuning constants

    public static double bucketPickup = 0.6;
    public static double bucketPlace = 0.45;
    public static double bucketUp = 0.9;

    public static double specimenWristPlace = 0.20; // towards inside of robot - change name later?
    public static double specimenWristPickup = 0.82;

    public static double POS_SAMPLE_PITCH_DROP_BUCKET = 0.2;
    public static double POS_SAMPLE_PITCH_ARM_DOWN = 0.4;
    public static double POS_SAMPLE_PITCH_PICKUP_READY = 0.43;
    public static double POS_SAMPLE_PITCH_PICKUP = 0.65;

    public static double POS_SAMPLE_YAW_LEFT_MAX = 0.3;
    public static double POS_SAMPLE_YAW_LEFT2 = 0.3;
    public static double POS_SAMPLE_YAW_LEFT1 = 0.4;
    public static double POS_SAMPLE_YAW_CENTER = 0.5;
    public static double POS_SAMPLE_YAW_RIGHT1 = 0.6;
    public static double POS_SAMPLE_YAW_RIGHT2 = 0.7;
    public static double POS_SAMPLE_YAW_RIGHT_MAX = 0.8;

    public static double sampleClawOpenWide = 0.95;
    public static double sampleClawOpen = 0.85;
    public static double sampleClawClosed = 0.74;

    public static double intakeWristGroundPosition = 0.7;
    public static double intakeWristUpPosition = 0.43;

    //vertical slide positions
    public static int highChamberBelowPos = 1750;
    public static int highChamberPlacePos = 2300;
    public static int barPlace1Pos = 950;
    public static int basketLowPos = 2450;
    public static int basketHighPos = 4380;
    public static int armGroundPos = 0;

    // calibrated for torque servo
    public static double specimenClawOpen = 0.94;
    public static double specimenClawClosed = 0.83;
    public static double specimenClawClosedTight = specimenClawClosed - 0.03;

    // Variables to track servo position
    private double sampleYawCurrentPosition;
    private double intakeWristCurrentPosition;

    @Override
    public void runOpMode() throws InterruptedException {
    }

    public void initControls() {

        // INITIALIZE SENSORS

        // Initialize 2 Deadwheel odometry
        configure2DeadWheel();

        // INITIALIZE MOTORS
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor1");
        omniMotor2 = initializeMotor("omniMotor2");
        omniMotor3 = initializeMotor("omniMotor3");

        omniMotor0.setDirection(DcMotor.Direction.REVERSE);
        omniMotor1.setDirection(DcMotor.Direction.FORWARD);
        omniMotor2.setDirection(DcMotor.Direction.FORWARD);
        omniMotor3.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("FTC 19168 Robot Initialization Done!");
        telemetry.update();
    }


    public double getVoltage() {
        return (hardwareMap.voltageSensor.iterator().next().getVoltage());
    }

    public void print(String Name, Object message) {
        //saves a line for quick debug messages
        telemetry.addData(Name, message);
        //telemetry.update();
    }

    public double relativePower(double intended_power) {
        //makes sure the power going to the motors is constant over battery life
        return (13 * intended_power) / getVoltage();
    }

    public DcMotor initializeMotor(String name) {
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    private void configure2DeadWheel() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(0, 165); //these are tuned for 2/5/2025 robot

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); //TODO check and fix these values


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Y offset", odo.getXOffset());
        telemetry.addData("X offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
    }


    public double getHypotenuse(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public double getHypotenuse(double x, double y, double z) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}