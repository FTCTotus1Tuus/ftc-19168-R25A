package org.firstinspires.ftc.teamcode.team.fsm;

public class ShootArtifactFSM {

    public enum ShootingStage {
        IDLE,
        ELEVATOR_UP,
        SHOTGUN_SPINUP,
        FEEDER_UP,
        FINISHED
    }

    private final DarienOpModeFSM opMode;

    private ShootingStage shootingStage = ShootingStage.IDLE;
    private double shootingStartTime = 0;

    // Timings (seconds)
    private static final double STAGE1_DELAY = .100;    // elevator up -> shotgun start
    private static final double STAGE2_DELAY = .600;    // shotgun running before feeder
    private static final double STAGE3_DELAY = .500;    // feeder up while spinning

    public ShootArtifactFSM(DarienOpModeFSM opMode) {
        this.opMode = opMode;
    }

    // Call this to begin shooting
    public void startShooting() {
        shootingStage = ShootingStage.ELEVATOR_UP;
        shootingStartTime = opMode.getRuntime();

        opMode.Elevator.setPosition(DarienOpModeFSM.ELEVATOR_POS_UP);
    }

    // Call this inside loop() or inside your main auto while-loop
    public void updateShooting(double shootingPower) {
        if (shootingStage == ShootingStage.IDLE ||
                shootingStage == ShootingStage.FINISHED) {
            return;
        }

        double elapsed = opMode.getRuntime() - shootingStartTime;

        switch (shootingStage) {

            case ELEVATOR_UP:
                if (elapsed >= STAGE1_DELAY) {
                    shotGun(DarienOpModeFSM.SHOT_GUN_POWER_UP * shootingPower);
                    shootingStage = ShootingStage.SHOTGUN_SPINUP;
                }
                break;

            case SHOTGUN_SPINUP:
                if (elapsed >= STAGE1_DELAY + STAGE2_DELAY) {
                    opMode.Feeder.setPosition(DarienOpModeFSM.FEEDER_POS_UP);
                    shootingStage = ShootingStage.FEEDER_UP;
                }
                break;

            case FEEDER_UP:
                if (elapsed >= STAGE1_DELAY + STAGE2_DELAY + STAGE3_DELAY) {
                    shotGunStop();
                    opMode.Feeder.setPosition(DarienOpModeFSM.FEEDER_POS_DOWN);
                    opMode.Elevator.setPosition(DarienOpModeFSM.ELEVATOR_POS_DOWN);
                    shootingStage = ShootingStage.FINISHED;
                }
                break;
        }
    }

    // Use this to check if it is done
    public boolean shootingDone() {
        return shootingStage == ShootingStage.FINISHED;
    }

    // If you want a hard reset:
    public void resetShooting() {
        shootingStage = ShootingStage.IDLE;
    }

    public void shotGun(double power) {
        opMode.ejectionMotorLeft.setPower(power);
        opMode.ejectionMotorRight.setPower(-power);
    }

    public void shotGunStop() {
        opMode.ejectionMotorLeft.setPower(0);
        opMode.ejectionMotorRight.setPower(0);
    }

}
