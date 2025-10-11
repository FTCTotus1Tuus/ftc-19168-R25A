package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp//(name="DC", group="DriverControl")
@Config
public class NewDriverControlOpMode extends DarienOpModeTeleop {
    // tuning constants for gobilda 117 rpm motor

    // tuning constants for gobilda 312 rpm motor and 4 stage long gobilda viper slide
    public static double INTAKE_SERVO_POS_UP = 0.67;
    public static double INTAKE_SERVO_POS_DOWN = 0.21;
    public static double TRAY_POS_1_INTAKE = 0.4;
    public static double TRAY_POS_2_INTAKE = 0;
    public static double TRAY_POS_3_INTAKE = 0.8;
    public static double TRAY_POS_1_SCORE = 1;
    public static double TRAY_POS_2_SCORE = 0.6;
    public static double TRAY_POS_3_SCORE = 0.2;
    double IntakeServoPosition = 0;
    double startTimeIntakeServo = 0;
    boolean isIntakeServoMoving = false;


    public double getIntakeServoPosition() {
        return IntakeServoPosition;
    }
    public void setIntakeServoPosition(double position) {
        IntakeServo.setPosition(position);
        IntakeServoPosition = position;
    }
    public void servoIncremental(Servo servo, double endPos, double startPos, double endDuration) {
        //calculate how many increments it will take to reach to position in the target time
        double currentPos;
        double startTime = getRuntime();
        double currentTime = startTime;
        while (currentTime - startTime < endDuration) {
            currentPos = ((endPos - startPos) / (endDuration - (currentTime - startTime))) * (currentTime - startTime) + startPos;
            servo.setPosition(currentPos);
            telemetry.addData("currentPos:", currentPos);
            telemetry.addData("currentTime:", currentTime);

            if (currentPos >= endPos){
                sleep(1000);
                break;
            }
            currentTime = getRuntime();

        }
        sleep(3000);
    }


    @Override
    public void runOpMode() {
        initControls();
        waitForStart();
        //Start

        while (this.opModeIsActive()) {
            //pollSensors();
            runDriveSystem();
          //assigning the ejectionmotorleft/right controls
            if (gamepad2.right_bumper){
                ejectionMotorLeft.setPower(1);
                ejectionMotorRight.setPower(-1);
            } else {
                ejectionMotorLeft.setPower(0);
                ejectionMotorRight.setPower(0);
            }

            /* TODO: DISABLE SPINNER UNTIL WE SWITCH IT TO A POSITIONAL SERVO
            // CONTROL: SPINNER
            if (gamepad2.left_bumper){
                Spinner.setPower(1);
            } else {
                Spinner.setPower(0);
            }

             */

            // CONTROL: INTAKE
            //classify the function
            //when g2.a button is pressed intake servo goes up in increments in relation to the time
            if (gamepad2.a){
                servoIncremental(IntakeServo, INTAKE_SERVO_POS_UP, INTAKE_SERVO_POS_DOWN, 1);
            } else {
                IntakeServo.setPosition(INTAKE_SERVO_POS_DOWN);
            }
            /*
            if (isIntakeServoMoving) {
                // Continue moving the servo until the position is reached.
                if (getIntakeServoPosition() < IntakeServoPosUp && getRuntime()-startTimeIntakeServo < 4) {
                    setIntakeServoPosition(getIntakeServoPosition() + 0.005);
                } else {
                    isIntakeServoMoving = false;
                }
            } else {
                if (gamepad2.a) {
                    // Start lifting the intake.
                    startTimeIntakeServo = getRuntime();
                    isIntakeServoMoving = true;
                } else {
                    IntakeServo.setPosition(IntakeServoPosDown);
                }
            }
            */
            // CONTROL: ROTATING TRAY
            if (gamepad2.dpad_left){
                Tray.setPosition(TRAY_POS_1_INTAKE);
            } else if (gamepad2.x){
                Tray.setPosition(TRAY_POS_1_SCORE);
            } else if (gamepad2.dpad_up){
                Tray.setPosition(TRAY_POS_2_INTAKE);
            } else if (gamepad2.y){
                Tray.setPosition(TRAY_POS_2_SCORE);
            } else if (gamepad2.dpad_right){
                Tray.setPosition(TRAY_POS_3_INTAKE);
            } else if (gamepad2.b){
                Tray.setPosition(TRAY_POS_3_SCORE);
            }

        }
    }
}