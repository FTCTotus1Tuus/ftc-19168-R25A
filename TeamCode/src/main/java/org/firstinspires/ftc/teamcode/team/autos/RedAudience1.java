package org.firstinspires.ftc.teamcode.team.autos;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(name = "RedAudience1", group = "Meet1")
@Config
public class RedAudience1 extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {

        initControls();
        waitForStart();
        if (isStopRequested()) return;

        //displayTrayTelemetry();

        //move to the desired position
        moveXY(70, 0, .3);
        Elevator.setPosition(ELEVATOR_POS_DOWN);
        Feeder.setPosition(FEEDER_POS_DOWN);
        servoIncremental(TrayServo,TRAY_POS_2_SCORE,currentTrayPosition, 1,4);
        //TrayServo.setPosition(TRAY_POS_2_SCORE);
        currentTrayPosition = TRAY_POS_2_SCORE;


        //read obelisk
        ArrayList<AprilTagDetection> currentDetections = null;
        double startTime = getRuntime();
        do {
            currentDetections = aprilTag.getDetections();
            telemetry.addLine("detecting apriltags...");
            telemetry.update();
        }
        while (currentDetections.isEmpty() || getRuntime() - startTime < TIMEOUT_APRILTAG_DETECTION);
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.update();
        waitForMotors(true);
        encoderRotate(Math.toRadians(40), .5, true);
        waitForMotors(true);

        // ASSUMES THAT GREEN IS PRELOADED IN POSITION 2
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                switch (detection.id) {
                    case 21:
                        telemetry.addData("Motif", "GPP");
                        // telemetryAprilTag(); //TODO: use this telemetry rather than the line above.
                        // shoot green
                        servoIncremental(TrayServo, TRAY_POS_2_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_2_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        // shoot purple
                        servoIncremental(TrayServo, TRAY_POS_3_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_3_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        // shoot purple
                        servoIncremental(TrayServo, TRAY_POS_1_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_1_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        break;
                    case 22:
                        telemetry.addData("Motif", "PGP");
                        //shoot purple
                        servoIncremental(TrayServo, TRAY_POS_1_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_1_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        //shoot green
                        servoIncremental(TrayServo, TRAY_POS_2_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_2_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        //shoot purple
                        servoIncremental(TrayServo, TRAY_POS_3_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_3_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        break;
                    case 23:
                        telemetry.addData("Motif", "PPG");
                        //shoot purple
                        servoIncremental(TrayServo, TRAY_POS_1_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_1_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        //shoot purple
                        servoIncremental(TrayServo, TRAY_POS_3_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_3_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        //shoot green
                        servoIncremental(TrayServo, TRAY_POS_2_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_2_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        break;
                }

                //return;
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        /*
        telemetry.addData("omnimotor 0: ", omniMotor0.getCurrentPosition());
        telemetry.addData("omnimotor 1: ", omniMotor1.getCurrentPosition());
        telemetry.addData("omnimotor 2: ", omniMotor2.getCurrentPosition());
        print("omnimotor 3: ", omniMotor3.getCurrentPosition());

         */
        /*
        //move to the desired position
        //moveXY(70, 0, .3);
        //TrayServo.setPosition(TRAY_POS_2_SCORE);
       // waitForMotors(true);
       // encoderRotate(Math.toRadians(40), .5, true);
        //waitForMotors(true);

        //shoot aritfact 1
        //displayTrayTelemetry();
       shootArtifact();
        sleep(500);
        //setBreakpoint();

        //shoot artifact 2
        //displayTrayTelemetry();
        servoIncremental(TrayServo,TRAY_POS_3_SCORE,currentTrayPosition, 1, 4);
        currentTrayPosition = TRAY_POS_3_SCORE;
        sleep(1000);
        //displayTrayTelemetry();
        shootArtifact();
        sleep(500);
        //setBreakpoint();

        //shoot artifact 3
        servoIncremental(TrayServo,TRAY_POS_1_SCORE,currentTrayPosition, 1,4);
        currentTrayPosition = TRAY_POS_1_SCORE;
        sleep(1000);
       shootArtifact();
        sleep(1000);
        //automaticIntake();
        */

        //move to pickup mid
        encoderRotate(Math.toRadians(-40), .5, true);
        waitForMotors(true);
        moveXY(-24, 0, .3);
        waitForMotors(true);
        encoderRotate(Math.toRadians(105), .5, true);
        waitForMotors(true);
        moveXY(4, 0, .5);
        waitForMotors(true);
    }

    private void displayTrayTelemetry() {
        tp.put("currentTrayPosition",currentTrayPosition);
        tp.put("currentTime",getRuntime());
        dash.sendTelemetryPacket(tp);
    }

}
