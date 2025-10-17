package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp//(name="DC", group="DriverControl")
@Config
public class NewDriverControlOpMode extends DarienOpModeTeleop {
    // tuning constants for gobilda 117 rpm motor

    // tuning constants for gobilda 312 rpm motor and 4 stage long gobilda viper slide
    public static double INTAKE_SERVO_POS_UP = 0.67;
    public static double INTAKE_SERVO_POS_DOWN = 0.21;
    public static double TRAY_POS_1_INTAKE = 0.35;
    public static double TRAY_POS_2_INTAKE = 0;
    public static double TRAY_POS_3_INTAKE = 0.8;
    public static double TRAY_POS_1_SCORE = 1;
    public static double TRAY_POS_2_SCORE = 0.6;
    public static double TRAY_POS_3_SCORE = 0.13;
    double IntakeServoPosition = 0;
    double startTimeIntakeServo = 0;
    boolean isIntakeServoMoving = false;
    public double currentTrayPosition ;
    public static double INTAKE_DISTANCE = 5;//in CM
    public static double ELEVATOR_POS_UP = 0.87;
    public static double ELEVATOR_POS_DOWN = 0.5;
    public static double FEEDER_POS_UP = .9;
    public static double FEEDER_POS_DOWN = .45;
    TelemetryPacket tp;
    FtcDashboard dash;

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
        //double Last_Time = currentTime;
        double ActualPos = 0;
        while (currentTime - startTime < endDuration) {
            if(endPos > startPos){
                // rotate tray clockwise
                currentPos = ((endPos - startPos) / (endDuration - (currentTime - startTime))) * (currentTime - startTime) + startPos;
            } else {
                // rotate tray counterclockwise
                currentPos = ((startPos - endPos) / (endDuration - (currentTime - startTime))) * (currentTime - startTime) + endPos;
            }
            servo.setPosition(currentPos);
            /*
            if (currentTime - Last_Time >= 0.240 ){
                servo.setPosition(currentPos);
                ActualPos = currentPos;
                Last_Time = currentTime;
            }

             */
            telemetry.addData("currentPos:", currentPos);
            telemetry.addData("currentTime:", currentTime);
            telemetry.update();
            tp.put("currentServo",currentPos);
            tp.put("currentTime",currentTime);
            //tp.put("lastTime",Last_Time);
            tp.put("ActPos",ActualPos);

            dash.sendTelemetryPacket(tp);

            if (currentPos >= endPos) {
                //sleep(1000);
                return;
            }
            currentTime = getRuntime();

        }
        //sleep(3000);
    }


    @Override
    public void runOpMode() {
        float gain = 2;
        initControls();
        // TODO: Add slowdown for tray init to POS 1 INTAKE
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();
        double startTimeColor = getRuntime();
        waitForStart();
        //Start
        while (this.opModeIsActive()) {
            //pollSensors();
            runDriveSystem();
            NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
          //assigning the ejectionmotorleft/right controls
            if (gamepad2.right_trigger > 0.05){
                ejectionMotorLeft.setPower(1);
                ejectionMotorRight.setPower(-1);
            } else {
                ejectionMotorLeft.setPower(0);
                ejectionMotorRight.setPower(0);
            }
            //CONTROL: TRAYINIT
            if (gamepad2.start) {
                servoIncremental(TrayServo, TRAY_POS_1_INTAKE, currentTrayPosition, 1);
            }

            //CONTROL: ELEVATOR
            if (gamepad2.left_bumper){
                Elevator.setPosition(ELEVATOR_POS_UP);
            } else {
                Elevator.setPosition(ELEVATOR_POS_DOWN);
            }
            //CONTROL: FEEDER
            if (gamepad2.right_bumper){
                Feeder.setPosition(FEEDER_POS_UP);
            } else {
                Feeder.setPosition(FEEDER_POS_DOWN);
            }
            // CONTROL: INTAKE
            //classify the function
            //when g2.a button is pressed intake servo goes up in increments in relation to the time
            // Update the gain value if either of the A or B gamepad buttons is being held
            if (gamepad1.a) {
                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                gain += 0.005;
            } else if (gamepad1.b && gain > 1) {
                // A gain of less than 1 will make the values smaller, which is not helpful.
                gain -= 0.005;
            }
            telemetry.addData("Gain", gain);
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            if (intakeColorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM));
                if (((DistanceSensor)intakeColorSensor).getDistance(DistanceUnit.CM) <= INTAKE_DISTANCE && (getRuntime()-startTimeColor) >= 1){
                    startTimeColor = getRuntime();
                    servoIncremental(IntakeServo, INTAKE_SERVO_POS_UP, INTAKE_SERVO_POS_DOWN, 1);
                }
            }
            telemetry.update();

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
                //Tray.setPosition(TRAY_POS_1_INTAKE);
                servoIncremental(TrayServo,TRAY_POS_1_INTAKE,currentTrayPosition, 1);
                currentTrayPosition = TRAY_POS_1_INTAKE;
            } else if (gamepad2.x){
                //Tray.setPosition(TRAY_POS_1_SCORE);
                servoIncremental(TrayServo,TRAY_POS_1_SCORE,currentTrayPosition, 1);
                currentTrayPosition = TRAY_POS_1_SCORE;
            } else if (gamepad2.dpad_up){
                //Tray.setPosition(TRAY_POS_2_INTAKE);
                servoIncremental(TrayServo,TRAY_POS_2_INTAKE,currentTrayPosition, 1);
                currentTrayPosition = TRAY_POS_2_INTAKE;
            } else if (gamepad2.y){
                //Tray.setPosition(TRAY_POS_2_SCORE);
                servoIncremental(TrayServo,TRAY_POS_2_SCORE,currentTrayPosition, 1);
                currentTrayPosition = TRAY_POS_2_SCORE;
            } else if (gamepad2.dpad_right){
                //Tray.setPosition(TRAY_POS_3_INTAKE);
                servoIncremental(TrayServo,TRAY_POS_3_INTAKE,currentTrayPosition, 1);
                currentTrayPosition = TRAY_POS_3_INTAKE;
            } else if (gamepad2.b){
                //Tray.setPosition(TRAY_POS_3_SCORE);
                servoIncremental(TrayServo,TRAY_POS_3_SCORE,currentTrayPosition, 1);
                currentTrayPosition = TRAY_POS_3_SCORE;
            }
            //MACRO: APRILTAG 21
            /*
            tray.setpostion(TRAY_POS_1_SCORE)
            if (gamepad.2(button combo)){
                servoincremental(TRAY_POS_2_SCORE)
                Elevator.setposition(ElevatorUp)
                ejectiomotorleft+right.setpower(1)
                Feeder.setposition(feederup)
                feeder.setposition(down)
                elevation.setposition(down)
                ejectionmotorleft+right.setpower(0)
                servoincermental(trayposition3score)
                Elevator.setposition(ElevatorUp)
                ejectiomotorleft+right.setpower(1)
                Feeder.setposition(feederup)
                feeder.setposition(down)
                elevation.setposition(down)
                ejectionmotorleft+right.setpower(0)
                servoincremental(Traypos1score)
                Elevator.setposition(ElevatorUp)
                ejectiomotorleft+right.setpower(1)
                Feeder.setposition(feederup)
                feeder.setposition(down)
                elevation.setposition(down)
                ejectionmotorleft+right.setpower(0)
            }
             */

        }
    }
}