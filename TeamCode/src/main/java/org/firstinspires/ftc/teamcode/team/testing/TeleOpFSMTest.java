package org.firstinspires.ftc.teamcode.team.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootArtifactFSM;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;

@TeleOp(name = "TeleopFSM Test", group = "DriverControl")
@Config
@Configurable
public class TeleOpFSMTest extends DarienOpModeFSM {

    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    public Follower follower;                   // Pedro Pathing follower instance
    //private ShootArtifactFSM shootArtifactFSM;

    boolean isRubberBandsReversed = false;
    public static double INTAKE_TIME = 1;

    @Override
    public void initControls() {
        super.initControls();
        follower = Constants.createFollower(hardwareMap);
        //shootArtifactFSM = new ShootArtifactFSM(this);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        float gain = 2;
        initControls();
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();
        double startTimeColor = getRuntime();

        waitForStart();
        if (isStopRequested()) return;
        //Start
        follower.startTeleopDrive(true);
        follower.update();

        while (this.opModeIsActive() && !isStopRequested()) {

            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
            //assigning the ejectionmotorleft/right controls
            if (gamepad1.y) {
                rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
                isRubberBandsReversed = false;
            } else if (gamepad1.a) {
                rubberBands.setPower(OUTPUT_RUBBER_BANDS_POWER);
                isRubberBandsReversed = true;
            } else if (gamepad1.x) {
                rubberBands.setPower(0);
                isRubberBandsReversed = false;
            }

            //if (gamepad2.back) {
            //    shootArtifact();
            //}
            if (gamepad2.right_trigger > 0.05) {
                ejectionMotorLeft.setPower(1);
                ejectionMotorRight.setPower(-1);
            } else {
                ejectionMotorLeft.setPower(0);
                ejectionMotorRight.setPower(0);
            }
            //CONTROL: TRAYINIT
            if (gamepad2.start) {
                servoIncremental(TrayServo, TRAY_POS_1_INTAKE, currentTrayPosition, 1, 1);
            }
            //CONTROL: EJECTIONMOTOR BACKWARDS
            if (gamepad2.left_trigger > 0.05) {
                ejectionMotorRight.setPower(.5);
                ejectionMotorLeft.setPower(-.5);
            }

            //CONTROL: ELEVATOR
            if (gamepad2.left_bumper) {
                Elevator.setPosition(ELEVATOR_POS_UP);
            } else {
                Elevator.setPosition(ELEVATOR_POS_DOWN);
            }
            //CONTROL: FEEDER
            if (gamepad2.right_bumper) {
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
                if (((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM) <= INTAKE_DISTANCE && (getRuntime() - startTimeColor) >= 1 && isRubberBandsReversed == false) {
                    startTimeColor = getRuntime();
                    servoIncremental(IntakeServo, INTAKE_SERVO_POS_UP, INTAKE_SERVO_POS_DOWN, INTAKE_TIME, 1);
                }
            }
            telemetry.update();

            if (gamepad2.a) {
                servoIncremental(IntakeServo, INTAKE_SERVO_POS_UP, INTAKE_SERVO_POS_DOWN, INTAKE_TIME, 1);
            } else {
                IntakeServo.setPosition(INTAKE_SERVO_POS_DOWN);
            }
            // CONTROL: ROTATING TRAY
            if (gamepad2.dpad_left) {
                servoIncremental(TrayServo, TRAY_POS_1_INTAKE, currentTrayPosition, 1, 4);
                currentTrayPosition = TRAY_POS_1_INTAKE;
            } else if (gamepad2.x) {
                servoIncremental(TrayServo, TRAY_POS_1_SCORE, currentTrayPosition, 1, 4);
                currentTrayPosition = TRAY_POS_1_SCORE;
            } else if (gamepad2.dpad_up) {
                servoIncremental(TrayServo, TRAY_POS_2_INTAKE, currentTrayPosition, 1, 4);
                currentTrayPosition = TRAY_POS_2_INTAKE;
            } else if (gamepad2.y) {
                servoIncremental(TrayServo, TRAY_POS_2_SCORE, currentTrayPosition, 1, 4);
                currentTrayPosition = TRAY_POS_2_SCORE;
            } else if (gamepad2.dpad_right) {
                servoIncremental(TrayServo, TRAY_POS_3_INTAKE, currentTrayPosition, 1, 4);
                currentTrayPosition = TRAY_POS_3_INTAKE;
            } else if (gamepad2.b) {
                servoIncremental(TrayServo, TRAY_POS_3_SCORE, currentTrayPosition, 1, 4);
                currentTrayPosition = TRAY_POS_3_SCORE;
            }

            /*
            // CONTROL: ROTATING TRAY USING FSM
            if (gamepad2.dpad_left && gamepad2.back) {
                setTrayPosition(TRAY_POS_1_INTAKE);
            } else if (gamepad2.x && gamepad2.back) {
                setTrayPosition(TRAY_POS_1_SCORE);
            } else if (gamepad2.dpad_up && gamepad2.back) {
                setTrayPosition(TRAY_POS_2_INTAKE);
            } else if (gamepad2.y && gamepad2.back) {
                setTrayPosition(TRAY_POS_2_SCORE);
            } else if (gamepad2.dpad_right && gamepad2.back) {
                setTrayPosition(TRAY_POS_3_INTAKE);
            } else if (gamepad2.b && gamepad2.back) {
                setTrayPosition(TRAY_POS_3_SCORE);
            }

             */

        }
    }
}