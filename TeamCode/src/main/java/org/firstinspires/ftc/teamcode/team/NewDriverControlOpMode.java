package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp//(name="DC", group="DriverControl")
@Config
public class NewDriverControlOpMode extends DarienOpModeTeleop {
    // tuning constants for gobilda 117 rpm motor

    // tuning constants for gobilda 312 rpm motor and 4 stage long gobilda viper slide
    public static double IntakeServoPosUp = 0.7;
    public static double IntakeServoPosDown = 0.2;

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
                IntakeServo.setPosition((IntakeServoPosDown));
            }

        }
    }
}