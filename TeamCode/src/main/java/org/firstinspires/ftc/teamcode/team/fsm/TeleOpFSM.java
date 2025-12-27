package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;

@TeleOp(name = "TeleopFSM", group = "DriverControl")
@Config
@Configurable
public class TeleOpFSM extends DarienOpModeFSM {
    //private Pose TARGET_LOCATION = new Pose();
    // INSTANCES
    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    public Follower follower;                   // Pedro Pathing follower instance

    // TUNING CONSTANTS
    public static double INTAKE_TIME = 1;
    public static double SHOT_TIMEOUT = 2.0; // seconds

    // VARIABLES
    private boolean isRubberBandsReversed = false;
    private double shotStartTime;
    private boolean shotStarted = false;
    private boolean isReadingAprilTag = false;
    private int pathState = 0; // drive state machine state
    private Timer pathTimer;

    @Override
    public void initControls() {
        super.initControls();
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();

        // Initialize rubber bands to off
        rubberBands.setPower(0);
        isRubberBandsReversed = false;
        //TrayServo.setPosition(currentTrayPosition); // Prevent movement at init
    }

    public int pathUpdate() {
        switch (pathState) {
            default:
            case 0:
                // Drive is in manual mode
                break;
            case 1:
                // Drive is controlled by pedro path
                if ((!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.0)) {
                    pathState = 0;
                }
                break;
        }
        return pathState;
    }

    /**
     *
     * @param angle
     * @return normalized angle to (-pi, pi]
     */
    private double normalizeRadians(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        float gain = 2;
        initControls();
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();
        startTimeIntakeColorSensor = getRuntime();

        waitForStart();
        if (isStopRequested()) return;
        //Start
        follower.startTeleopDrive(true);
        follower.update();

        while (this.opModeIsActive() && !isStopRequested()) {

            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
            pathState = pathUpdate();

            // -----------------
            // GAMEPAD1 CONTROLS
            // -----------------

            NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
            //assigning the ejectionmotorleft/right controls
            if (gamepad1.y) {
                rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
                isRubberBandsReversed = false;
            } else if (gamepad1.a && !gamepad1.start) {
                rubberBands.setPower(OUTPUT_RUBBER_BANDS_POWER);
                isRubberBandsReversed = true;
            } else if (gamepad1.x) {
                rubberBands.setPower(0);
                isRubberBandsReversed = false;
            }


            // -----------------
            // GAMEPAD2 CONTROLS
            // -----------------

            if (!shotStarted) {
                // -----------------
                // MANUAL CONTROLS: only allowed when not in shooting macro
                // -----------------

                //CONTROL: POINT ROBOT TO GOAL
                /*
                if (gamepad1.left_bumper) {

                    //point robot at blue goal if gamepad1 left bumper is pressed
                    aprilTagDetections = tagFSM.getDetections();
                    aprilTagDetections.removeIf(tag -> tag.id == 21 || tag.id == 22 || tag.id == 23 || tag.id == 24);
                   // TARGET_LOCATION = new Pose(follower.getPose().getX(), follower.getPose().getY(),aprilTagDetections.getFirst().ftcPose.bearing);

                    follower.pathBuilder().setLinearHeadingInterpolation(follower.getHeading(), aprilTagDetections.get(0).ftcPose.bearing);

                }

                 */
                if (gamepad1.right_trigger > 0.05 && !isReadingAprilTag) {
                    //point robot at red goal if gamepad1 right bumper is pressed
                    tagFSM.start(getRuntime());
                    isReadingAprilTag = true;

                } else if (isReadingAprilTag) {
                    tagFSM.update(getRuntime(), true, telemetry);
                    telemetry.addLine("Reading...");

                    if (tagFSM.isDone()) {
                        telemetry.addLine("DONE READING!");
                        isReadingAprilTag = false;
                        aprilTagDetections = tagFSM.getDetections();
                        //aprilTagDetections.removeIf(tag -> tag.id != 24);
                        aprilTagDetections.removeIf(tag -> tag.id == 20 || tag.id == 21 || tag.id == 22 || tag.id == 23);
                        if (!aprilTagDetections.isEmpty()) {
                            telemetry.addLine("FOUND APRILTAG!");
                            tagFSM.telemetryAprilTag(telemetry);
                            // Rotate the robot only if an apriltag is detected and it's the blue goal apriltag id
                            AprilTagDetection detection = aprilTagDetections.get(0);
                            PathChain rotatePath;
                            if (detection.id == 24) {
                                telemetry.addLine("ALIGNING TO GOAL...");
                                telemetry.addData("H1:", follower.getHeading());
                                telemetry.addData("H2:", detection.ftcPose.bearing);
                                double currentHeading = follower.getHeading();
                                double relativeHeading = detection.ftcPose.bearing;
                                double targetHeading = normalizeRadians(currentHeading + relativeHeading);
                                rotatePath = follower.pathBuilder()
                                        .addPath(
                                                new BezierPoint(new Pose(follower.getPose().getX(), follower.getPose().getY()))
                                        )
                                        .setLinearHeadingInterpolation(currentHeading, targetHeading)
                                        .build();

                                follower.followPath(rotatePath);


                                pathTimer.resetTimer();
                                pathState = 1;
                            } // end detection.id == 24
                        } // end detection is empty
                    } // end tagFSM is done

                } // end Red Goal April Tag

                //CONTROL: EJECTION MOTORS
                if (gamepad2.right_trigger > 0.05 && gamepad2.right_stick_y >= -0.05) {
                    ejectionMotorLeft.setPower(getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP));
                    ejectionMotorRight.setPower(-getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP));
                } else if (gamepad2.right_trigger > 0.05 && gamepad2.right_stick_y < -0.05) {
                    ejectionMotorLeft.setPower(getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP_FAR));
                    ejectionMotorRight.setPower(-getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP_FAR));
                } else if (gamepad2.left_trigger > 0.05) {
                    ejectionMotorLeft.setPower(-getVoltageAdjustedMotorPower(SHOT_GUN_POWER_DOWN));
                    ejectionMotorRight.setPower(getVoltageAdjustedMotorPower(SHOT_GUN_POWER_DOWN));
                } else {
                    ejectionMotorLeft.setPower(0);
                    ejectionMotorRight.setPower(0);
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
                if (gamepad2.a) {
                    IntakeServo.setPosition(INTAKE_SERVO_POS_UP);
                    intakeLifted = false; // Cancel any automatic lift
                } else if (intakeColorSensor instanceof DistanceSensor) {
                    telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM));
                    if (!intakeLifted && ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM) <= INTAKE_DISTANCE && !isRubberBandsReversed && (getRuntime() - startTimeIntakeColorSensor) >= COLOR_SENSOR_TIMEOUT) {
                        startTimeIntakeColorSensor = getRuntime();
                        intakeLifted = true;
                        intakeLiftStartTime = getRuntime();
                    }
                    if (intakeLifted) {
                        IntakeServo.setPosition(INTAKE_SERVO_POS_UP);
                        if (getRuntime() - intakeLiftStartTime >= INTAKE_SERVO_DURATION_RAISE) {
                            intakeLifted = false;
                        }
                    } else {
                        IntakeServo.setPosition(INTAKE_SERVO_POS_DOWN);
                    }
                } else {
                    IntakeServo.setPosition(INTAKE_SERVO_POS_DOWN);
                }

                // CONTROL: ROTATING TRAY
                if (gamepad2.dpad_left) {
                    setTrayPosition(TRAY_POS_1_INTAKE);
                    //servoIncremental(TrayServo, TRAY_POS_1_INTAKE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_1_INTAKE;
                } else if (gamepad2.x) {
                    setTrayPosition(TRAY_POS_1_SCORE);
                    //servoIncremental(TrayServo, TRAY_POS_1_SCORE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_1_SCORE;
                } else if (gamepad2.dpad_up) {
                    setTrayPosition(TRAY_POS_2_INTAKE);
                    //servoIncremental(TrayServo, TRAY_POS_2_INTAKE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_2_INTAKE;
                } else if (gamepad2.y) {
                    setTrayPosition(TRAY_POS_2_SCORE);
                    //servoIncremental(TrayServo, TRAY_POS_2_SCORE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_2_SCORE;
                } else if (gamepad2.dpad_right) {
                    setTrayPosition(TRAY_POS_3_INTAKE);
                    //servoIncremental(TrayServo, TRAY_POS_3_INTAKE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_3_INTAKE;
                } else if (gamepad2.b && !gamepad2.start) {
                    setTrayPosition(TRAY_POS_3_SCORE);
                    //servoIncremental(TrayServo, TRAY_POS_3_SCORE, currentTrayPosition, 1, 4);
                    //currentTrayPosition = TRAY_POS_3_SCORE;
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

                // -----------------
                // IMPORTANT: ALWAYS PUT MACRO CONTROLS AFTER MANUAL CONTROLS
                // -----------------

                //CONTROL: START SHOTGUN MACRO USING FSM
                if (gamepad2.dpad_down && gamepad2.right_trigger > 0.05) {
                    // TODO: pre-spin up the shotgun before starting the shooting FSM
                    if (gamepad2.right_stick_y < -0.05) {
                        shootArtifactFSM.startShooting(SHOT_GUN_POWER_UP_FAR);
                    } else {
                        shootArtifactFSM.startShooting(SHOT_GUN_POWER_UP);
                    }
                    shotStartTime = getRuntime();
                    shotStarted = true;
                }
                telemetry.addData("shotStarted", shotStarted);
                telemetry.addData("H1:", follower.getHeading());

            } //manual controls
            else {
                // -----------------
                // MACRO CONTROLS
                // -----------------

                //CONTROL: SHOTGUN MACRO
                shootArtifactFSM.updateShooting();
                if (shootArtifactFSM.shootingDone() || getRuntime() - shotStartTime >= SHOT_TIMEOUT) {
                    shootArtifactFSM.resetShooting();
                    shotStarted = false;
                }
                /*
                telemetry.addData("shotStarted", shotStarted);
                telemetry.addData("shootingStage", shootArtifactFSM.getShootingStage());
                telemetry.addData("shootingDone", shootArtifactFSM.shootingDone());
                telemetry.addData("Elevator Pos", Elevator.getPosition());
                telemetry.addData("Ejection L", ejectionMotorLeft.getPower());
                telemetry.addData("Ejection R", ejectionMotorRight.getPower());

                 */

            } //macro controls
            telemetry.update();

        } //while opModeIsActive
    } //runOpMode
} //TeleOpFSM class