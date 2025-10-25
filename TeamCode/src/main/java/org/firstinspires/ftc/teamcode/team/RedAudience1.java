package org.firstinspires.ftc.teamcode.team;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedAudience1", group = "Meet1")
@Config
public class RedAudience1 extends DarienOpModeAuto {
    @Override
    public void runOpMode() throws InterruptedException {

        initControls();
        waitForStart();
        if (isStopRequested()) return;

        //displayTrayTelemetry();

        Elevator.setPosition(ELEVATOR_POS_DOWN);
        Feeder.setPosition(FEEDER_POS_DOWN);
        servoIncremental(TrayServo,TRAY_POS_2_SCORE,currentTrayPosition, 1,4);
        //TrayServo.setPosition(TRAY_POS_2_SCORE);
        currentTrayPosition = TRAY_POS_2_SCORE;
        sleep(2000);

        /*
        telemetry.addData("omnimotor 0: ", omniMotor0.getCurrentPosition());
        telemetry.addData("omnimotor 1: ", omniMotor1.getCurrentPosition());
        telemetry.addData("omnimotor 2: ", omniMotor2.getCurrentPosition());
        print("omnimotor 3: ", omniMotor3.getCurrentPosition());

         */

        //move to the desired position
        sleep(250);
        moveXY(70, 0, .3);
        //TrayServo.setPosition(TRAY_POS_2_SCORE);
        waitForMotors(true);
        encoderRotate(Math.toRadians(40), .5, true);
        waitForMotors(true);

        //shoot aritfact 1
        //displayTrayTelemetry();
        Elevator.setPosition(ELEVATOR_POS_UP);
        sleep(100);
        shotGun(SHOT_GUN_POWER_DOWN);
        //start spinning down
        sleep(500);
        shotGun(SHOT_GUN_POWER_UP);
        sleep(400);
        //start spinning up
        Feeder.setPosition(FEEDER_POS_UP);
        //move feeder up while spinner is still spinning
        sleep(1500);
        shotGunStop();
        //stop spinning
        Feeder.setPosition(FEEDER_POS_DOWN);
        Elevator.setPosition(ELEVATOR_POS_DOWN);
        sleep(1000);
        //setBreakpoint();

        //shoot artifact 2
        //displayTrayTelemetry();

        servoIncremental(TrayServo,TRAY_POS_3_SCORE,currentTrayPosition, 1, 4);
        currentTrayPosition = TRAY_POS_3_SCORE;
        sleep(2000);

        //displayTrayTelemetry();

        Elevator.setPosition(ELEVATOR_POS_UP);
        sleep(100);
        shotGun(SHOT_GUN_POWER_DOWN);
        //start spinning down
        sleep(500);
        shotGun(SHOT_GUN_POWER_UP);
        sleep(400);
        //start spinning up
        Feeder.setPosition(FEEDER_POS_UP);
        //move feeder up while spinner is still spinning
        sleep(1500);
        shotGunStop();
        //stop spinning
        Feeder.setPosition(FEEDER_POS_DOWN);
        Elevator.setPosition(ELEVATOR_POS_DOWN);
        sleep(1000);
        //setBreakpoint();

        //shoot artifact 3
        servoIncremental(TrayServo,TRAY_POS_1_SCORE,currentTrayPosition, 1,4);
        currentTrayPosition = TRAY_POS_1_SCORE;
        sleep(2000);
        Elevator.setPosition(ELEVATOR_POS_UP);
        sleep(100);
        shotGun(SHOT_GUN_POWER_DOWN);
        //start spinning down
        sleep(500);
        shotGun(SHOT_GUN_POWER_UP);
        sleep(400);
        //start spinning up
        Feeder.setPosition(FEEDER_POS_UP);
        //move feeder up while spinner is still spinning
        sleep(1500);
        shotGunStop();
        //stop spinning
        Feeder.setPosition(FEEDER_POS_DOWN);
        Elevator.setPosition(ELEVATOR_POS_DOWN);
        sleep(1000);

        //move back to park
        encoderRotate(Math.toRadians(-40), .5, true);
        waitForMotors(true);
        moveXY(-60, 0, .3);
        waitForMotors(true);
        moveXY(0, -12, .5);
        waitForMotors(true);
    }

    private void displayTrayTelemetry() {
        tp.put("currentTrayPosition",currentTrayPosition);
        tp.put("currentTime",getRuntime());
        dash.sendTelemetryPacket(tp);
    }

}
