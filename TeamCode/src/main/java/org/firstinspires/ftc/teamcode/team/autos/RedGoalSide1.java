package org.firstinspires.ftc.teamcode.team.autos;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(name = "RedGoalSide1", group = "Meet1")
@Config
public class RedGoalSide1 extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {

        initControls();
        waitForStart();
        if (isStopRequested()) return;

        //resetting all positions
        Elevator.setPosition(ELEVATOR_POS_DOWN);
        Feeder.setPosition(FEEDER_POS_DOWN);
        servoIncremental(TrayServo,TRAY_POS_2_SCORE,currentTrayPosition, 1,4);
        currentTrayPosition = TRAY_POS_2_SCORE;

        //move backwards to read obelisk
        moveXY(-26, 0, .25);
        //sleep so cam can read apriltag
        //sleep(2000);

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
        moveXY(6, 0, 0.25);
        waitForMotors(true);
        encoderRotate(Math.toRadians(75), .5, true);
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
        //after shooting 3 artifacts, move to park intake facing red goal for teleop start
        encoderRotate(Math.toRadians(20), .5, true);
        waitForMotors(true);
        moveXY(0, 15, 0.4);
        waitForMotors(true);



    }
}