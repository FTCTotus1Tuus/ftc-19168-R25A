package org.firstinspires.ftc.teamcode.team.autos;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.DarienOpModeAuto;

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
        waitForMotors(true);
        //sleep so cam can read apriltag
        sleep(2000);
        encoderRotate(Math.toRadians(65),.5,true);
        waitForMotors(true);

        //read obelisk


        //shoot aritfact 1
        //displayTrayTelemetry();
        Elevator.setPosition(ELEVATOR_POS_UP);
        //shotGun(SHOT_GUN_POWER_DOWN);
        //start spinning down
        sleep(100);
        shotGun(SHOT_GUN_POWER_UP);
        sleep(600);
        //start spinning up
        Feeder.setPosition(FEEDER_POS_UP);
        //move feeder up while spinner is still spinning
        sleep(400);
        shotGunStop();
        //stop spinning
        Feeder.setPosition(FEEDER_POS_DOWN);
        Elevator.setPosition(ELEVATOR_POS_DOWN);
        sleep(500);
        //setBreakpoint();

        //shoot artifact 2
        //displayTrayTelemetry();
        servoIncremental(TrayServo,TRAY_POS_3_SCORE,currentTrayPosition, 1, 4);
        currentTrayPosition = TRAY_POS_3_SCORE;
        sleep(1000);
        //displayTrayTelemetry();
        Elevator.setPosition(ELEVATOR_POS_UP);
        //shotGun(SHOT_GUN_POWER_DOWN);
        //start spinning down
        sleep(100);
        shotGun(SHOT_GUN_POWER_UP);
        sleep(600);
        //start spinning up
        Feeder.setPosition(FEEDER_POS_UP);
        //move feeder up while spinner is still spinning
        sleep(400);
        shotGunStop();
        //stop spinning
        Feeder.setPosition(FEEDER_POS_DOWN);
        Elevator.setPosition(ELEVATOR_POS_DOWN);
        sleep(500);
        //setBreakpoint();

        //shoot artifact 3
        servoIncremental(TrayServo,TRAY_POS_1_SCORE,currentTrayPosition, 1,4);
        currentTrayPosition = TRAY_POS_1_SCORE;
        sleep(1000);
        Elevator.setPosition(ELEVATOR_POS_UP);
        //shotGun(SHOT_GUN_POWER_DOWN);
        //start spinning down
        sleep(100);
        shotGun(SHOT_GUN_POWER_UP);
        sleep(400);
        //start spinning up
        Feeder.setPosition(FEEDER_POS_UP);
        //move feeder up while spinner is still spinning
        sleep(500);
        shotGunStop();
        //stop spinning
        Feeder.setPosition(FEEDER_POS_DOWN);
        Elevator.setPosition(ELEVATOR_POS_DOWN);
        sleep(1000);
        //automaticIntake();
    }
}