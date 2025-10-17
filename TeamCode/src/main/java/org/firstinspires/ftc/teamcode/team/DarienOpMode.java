// To run on the robot:
// 1. Make Module   (Top ribbon)
// 2. Run           (Top ribbon)

package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DarienOpMode extends LinearOpMode {

    // HARDWARE COMPONENTS
    public DcMotor omniMotor0; // left front
    public DcMotor omniMotor1; // right front
    public DcMotor omniMotor2; // left rear
    public DcMotor omniMotor3; // right rear
   // public DcMotor slideMotor1;
   // public DcMotor tiltMotor;
   // public Servo claw;
    public Servo TrayServo;
    public Servo Elevator;
    public Servo Feeder;
    public Servo IntakeServo;
    public DcMotor ejectionMotorRight;
    public DcMotor ejectionMotorLeft;
    public NormalizedColorSensor intakeColorSensor;

    //    public IMU imu;
   // public GoBildaPinpointDriver odo;

    // HARDWARE FIXED CONSTANTS
    public static double encoderResolution = 537.7; //no change unless we change motors
    public double wheelDiameter = 3.75; // inches
    public double constMult = (wheelDiameter * (Math.PI));
    public static double rotationTolerance = 2; //in degrees
    public double inchesToEncoder = encoderResolution / constMult;
    public static double PI = 3.1416;


    // HARDWARE TUNING CONSTANTS
    public int encoderPos0, encoderPos1, encoderPos2, encoderPos3;
    public int encoderPos;
    public double regularDivBy = 2;
    public double turboDivBy = 1;
    public boolean turboBoost;

    @Override
    public void runOpMode() throws InterruptedException {
    }

    public void initControls() {

        // INITIALIZE SENSORS

        // Initialize 2 Deadwheel odometry
       // configure2DeadWheel();

        // INITIALIZE SERVOS
        //claw = hardwareMap.get(Servo.class, "claw");
        TrayServo = hardwareMap.get(Servo.class, "Tray");
        Elevator = hardwareMap.get(Servo.class, "Elevator");
        IntakeServo = hardwareMap.get(Servo.class, "intakeServo");
        Feeder = hardwareMap.get(Servo.class, "Feeder");
        // INITIALIZE SENSORS
        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColorSensor");
        // INITIALIZE MOTORS
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor1");
        omniMotor2 = initializeMotor("omniMotor2");
        omniMotor3 = initializeMotor("omniMotor3");
        ejectionMotorRight = initializeMotor("ejectionMotorRight");
        ejectionMotorLeft = initializeMotor("ejectionMotorLeft");

        omniMotor0.setDirection(DcMotor.Direction.FORWARD);
        omniMotor1.setDirection(DcMotor.Direction.REVERSE);
        omniMotor2.setDirection(DcMotor.Direction.REVERSE);
        omniMotor3.setDirection(DcMotor.Direction.FORWARD);

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
