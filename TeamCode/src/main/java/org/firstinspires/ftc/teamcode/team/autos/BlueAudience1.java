package org.firstinspires.ftc.teamcode.team.autos;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(name = "Blue Audience 1", group = "Blues")
@Config
public class BlueAudience1 extends DarienOpModeAuto {
    ArrayList<AprilTagDetection> Motif;
    @Override
    public void runOpMode() throws InterruptedException {

        initControls();
        waitForStart();
        if (isStopRequested()) return;

        //displayTrayTelemetry();

        //move to the desired position
        moveXY(70, 0, AUTO_MOVE_POWER);
        Elevator.setPosition(ELEVATOR_POS_DOWN);
        Feeder.setPosition(FEEDER_POS_DOWN);
        servoIncremental(TrayServo,TRAY_POS_2_SCORE,currentTrayPosition, 1,4);
        //TrayServo.setPosition(TRAY_POS_2_SCORE);
        currentTrayPosition = TRAY_POS_2_SCORE;
        sleep(1000);

        //read obelisk
        Motif = readAprilTagSequence();
        waitForMotors(true);
        encoderRotate(Math.toRadians(-40), AUTO_ROTATATE_POWER, true);
        waitForMotors(true);

        // ASSUMES THAT GREEN IS PRELOADED IN POSITION 2
        // TODO: Compare this to RedGoalSide1.java and create new functions to encapsulate any code that is identical.
        shootApriltagSequence(Motif);

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
        //waitForMotors(true);
        //encoderRotate(Math.toRadians(-40), .5, true);
        //waitForMotors(true);

        //shoot aritfact 1
        //displayTrayTelemetry();
        //shootArtifact();
        //sleep(500);
        //setBreakpoint();

        //shoot artifact 2
        //displayTrayTelemetry();
        //servoIncremental(TrayServo,TRAY_POS_3_SCORE,currentTrayPosition, 1, 4);
        //currentTrayPosition = TRAY_POS_3_SCORE;
        //sleep(1000);
        //displayTrayTelemetry();
        //shootArtifact();
        //sleep(500);
        //setBreakpoint();

        //shoot artifact 3
        //servoIncremental(TrayServo,TRAY_POS_1_SCORE,currentTrayPosition, 1,4);
        //currentTrayPosition = TRAY_POS_1_SCORE;
        //sleep(1000);
        //shootArtifact();
        //sleep(1000);
        //automaticIntake();
        */

        //move to pickup mid
        encoderRotate(Math.toRadians(40), AUTO_ROTATATE_POWER, true);
        waitForMotors(true);
        moveXY(-24, 0, AUTO_MOVE_POWER);
        waitForMotors(true);
        encoderRotate(Math.toRadians(-105), AUTO_ROTATATE_POWER, true);
        waitForMotors(true);
        moveXY(4, 0, AUTO_MOVE_POWER);
        waitForMotors(true);
    }

    private void displayTrayTelemetry() {
        tp.put("currentTrayPosition",currentTrayPosition);
        tp.put("currentTime",getRuntime());
        dash.sendTelemetryPacket(tp);
    }

}
