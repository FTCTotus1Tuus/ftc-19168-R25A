package org.firstinspires.ftc.teamcode.team.autos;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(name = "Red GoalSide 1", group = "Reds")
@Config
public class RedGoalSide1 extends DarienOpModeAuto {
    ArrayList<AprilTagDetection> Motif;
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
        moveXY(-26, 0, AUTO_MOVE_POWER);
        waitForMotors(true);
        //sleep so cam can read apriltag
        sleep(2000);
        //Read the apriltag sequence
        Motif = readAprilTagSequence();

        waitForMotors(true);
        moveXY(6, 0, AUTO_MOVE_POWER);
        waitForMotors(true);
        encoderRotate(Math.toRadians(75), AUTO_ROTATATE_POWER, true);
        waitForMotors(true);

        // ASSUMES THAT GREEN IS PRELOADED IN POSITION 2
        shootApriltagSequence(Motif);

        //after shooting 3 artifacts, move to park intake facing red goal for teleop start
        encoderRotate(Math.toRadians(20), AUTO_ROTATATE_POWER, true);
        waitForMotors(true);
        moveXY(0, 15, AUTO_MOVE_POWER);
        waitForMotors(true);
    }
}