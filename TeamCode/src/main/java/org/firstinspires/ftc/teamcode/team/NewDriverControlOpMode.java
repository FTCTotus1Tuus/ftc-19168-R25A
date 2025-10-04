package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp//(name="DC", group="DriverControl")
@Config
public class NewDriverControlOpMode extends DarienOpModeTeleop {
    // tuning constants for gobilda 117 rpm motor

    // tuning constants for gobilda 312 rpm motor and 4 stage long gobilda viper slide
    public static double IntakeServoPosUp = 0.7;
    public static double IntakeServoPosDown = 0.2;
    public static double TrayPos1Intake = 0;
    public static double TrayPos2Intake = 0.885;
    public static double TrayPos3Intake = 0.45;
    public static double TrayPos1Score = 0.7;
    public static double TrayPos2Score = 0.2;
    public static double TrayPos3Score = 0;

    @Override
    public void runOpMode() {
        initControls();
        waitForStart();
        //Start

        while (this.opModeIsActive()) {
            //pollSensors();
            runDriveSystem();
          //assigning the ejectionmotorleft/right controls
            if (gamepad2.right_bumper){
                ejectionMotorLeft.setPower(1);
                ejectionMotorRight.setPower(-1);
            } else {
                ejectionMotorLeft.setPower(0);
                ejectionMotorRight.setPower(0);
            }
            if (gamepad2.left_bumper){
                Spinner.setPower(1);
            } else {
                Spinner.setPower(0);
            }
            if (gamepad2.a){
                IntakeServo.setPosition(IntakeServoPosUp);
            } else {
                IntakeServo.setPosition(IntakeServoPosDown);
            }
            if (gamepad2.dpad_left && gamepad2.y){
                Tray.setPosition(TrayPos1Intake);
            } else if (gamepad2.dpad_left && gamepad2.x){
                Tray.setPosition(TrayPos1Score);
            } else if (gamepad2.dpad_up && gamepad2.y){
                Tray.setPosition(TrayPos2Intake);
            } else if (gamepad2.dpad_up && gamepad2.x){
                Tray.setPosition(TrayPos2Score);
            } else if (gamepad2.dpad_right && gamepad2.y){
                Tray.setPosition(TrayPos3Intake);
            } else if (gamepad2.dpad_right && gamepad2.x){
                Tray.setPosition(TrayPos3Score);
            }

        }
    }
}