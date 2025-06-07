package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp//(name="DC", group="DriverControl")
@Config
public class NewDriverControlOpMode extends DarienOpModeTeleop {
    @Override
    public void runOpMode() {
        initControls();
        waitForStart();
        //Start
        double tilt_power = 0, tilt_iduty = 0, tilt_pduty = 0;
        double tilt_pgain = .002;
        //double tilt_igain = .00001;
        double tilt_igain = .00000;
        double tilt_sp = 0;
        double T2=440;
        double T1=-25;

        while (this.opModeIsActive()) {

            //pollSensors();
            runDriveSystem();
            //runSlideMotorSystem();
            runMotorWithEncoderStops(slideMotor1, gamepad2.right_stick_y,"Slide",0.6, 0, 100, 2300, 100);
           // runMotorWithEncoderStops(tiltMotor, gamepad2.left_stick_y,"Tilt" ,-1, 0, 100, 2300, 100);

            //-25 to 440
            tilt_sp = clamp(gamepad2.left_stick_y * -1, 0.0, 1.0) * (T2-T1) + T1;

            //tilt_iduty = tilt_power;
            tilt_pduty = clamp(tilt_pgain * (tilt_sp - tiltMotor.getCurrentPosition()), -.5, 0.8);
            tilt_iduty = 0 * clamp(tilt_igain * (tilt_sp - tiltMotor.getCurrentPosition()) + tilt_iduty, -0.2, 1);
            tilt_power = clamp(tilt_pduty + tilt_iduty, 0.0, 1.0);

            tiltMotor.setPower(tilt_power);

            telemetry.addData("Pduty: ", tilt_pduty);
            telemetry.addData("Iduty: ", tilt_iduty);
            telemetry.addData("tiltPos: ", tiltMotor.getCurrentPosition());
            telemetry.addData("tiltsp: ", tilt_sp);

            telemetry.update();


        }
    }
}