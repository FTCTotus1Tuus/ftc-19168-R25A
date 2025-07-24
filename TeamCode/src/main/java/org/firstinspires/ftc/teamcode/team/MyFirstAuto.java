package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MyFirstAuto", group = "State")
public class MyFirstAuto extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {

        initControls();
        waitForStart();

        // Goal: Drive from wall to pickup sample from the floor and score it in the low basket.

        // Drive from wall to 3rd spike mark
        updatePosition();
        moveToPosition(0, 24, .2);
        waitForMotors();
        // Pick up sample from floor


    }
}