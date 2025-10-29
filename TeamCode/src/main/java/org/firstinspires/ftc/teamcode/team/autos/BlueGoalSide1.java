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

        //shoot artifact 1
        Elevator.setPosition(ELEVATOR_POS_UP);
        sleep(100);
        shotGun(SHOT_GUN_POWER_DOWN);
        //start spinning down
        sleep(500);
        shotGun(SHOT_GUN_POWER_UP_GOAL);
        sleep(400);
        //start spinning up
        Feeder.setPosition(FEEDER_POS_UP);
        //move feeder up while spinner is still spinning
        sleep(1500);
        shotGunStop();
        //stop spinning
        Feeder.setPosition(FEEDER_POS_DOWN);
        Elevator.setPosition(ELEVATOR_POS_DOWN);


    }
}