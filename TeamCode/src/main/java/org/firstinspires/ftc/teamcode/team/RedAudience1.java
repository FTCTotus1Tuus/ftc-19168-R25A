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

        telemetry.addData("omnimotor 0: ", omniMotor0.getCurrentPosition());
        telemetry.addData("omnimotor 1: ", omniMotor1.getCurrentPosition());
        telemetry.addData("omnimotor 2: ", omniMotor2.getCurrentPosition());
        print("omnimotor 3: ", omniMotor3.getCurrentPosition());

        //move to the desired position
        setBreakpoint();
        moveXY(10, 0, .3);
        waitForMotors(true);
        setBreakpoint();
        //encoderRotate(Math.toRadians(45), .5, true);
        /*
        TrayServo.setPosition(TRAY_POS_1_SCORE);
        servoIncremental(TrayServo, TRAY_POS_2_SCORE, TRAY_POS_1_SCORE, .5, 1);
        Elevator.setPosition(ELEVATOR_POS_UP);
        shotgun(ejectionMotor1, ejectionMotor2);
        Feeder.setposition(feederup);
        feeder.setposition(down);
        elevation.setposition(down);
        ejectionmotorleft+right.setpower(0);
        servoincermental(trayposition3score);
        Elevator.setposition(ElevatorUp);
        ejectiomotorleft+right.setpower(1);
        Feeder.setposition(feederup);
        feeder.setposition(down);
        elevation.setposition(down);
        ejectionmotorleft+right.setpower(0);
        servoincremental(Traypos1score);
        Elevator.setposition(ElevatorUp);
        ejectiomotorleft+right.setpower(1);
        Feeder.setposition(feederup);
        feeder.setposition(down);
        elevation.setposition(down);
        ejectionmotorleft+right.setpower(0);

         */

    }
}
