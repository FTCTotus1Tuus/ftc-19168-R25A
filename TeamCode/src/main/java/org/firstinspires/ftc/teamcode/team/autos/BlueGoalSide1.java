package org.firstinspires.ftc.teamcode.team.autos;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

@Autonomous(name = "BlueGoalSide1", group = "Meet1")
@Config
public class BlueGoalSide1 extends DarienOpModeAuto {
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
        waitForMotors(true);
        //sleep so cam can read apriltag
        sleep(2000);
        encoderRotate(Math.toRadians(-65),.5,true);
        waitForMotors(true);

        //shoot aritfact 1
        //displayTrayTelemetry();
        Elevator.setPosition(ELEVATOR_POS_UP);
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
    }
}

