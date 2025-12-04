package org.firstinspires.ftc.teamcode.team.autosPedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;

@Autonomous(name = "RedGoalSidePedro", group = "Pedro:Reds", preselectTeleOp = "Teleop")
@Configurable
public class RedGoalSide1 extends DarienOpModeFSM {

    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    public Follower follower;                   // Pedro Pathing follower instance
    private int pathState;                      // State machine state
    private Paths paths;                        // Paths
    private Timer pathTimer, opmodeTimer;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- ROBOT + HARDWARE INIT (from DarienOpModeFSM) ---
        initControls(); // sets up TrayServo, Elevator, Feeder, motors, AprilTag, etc.

        // --- PEDRO + TIMERS INIT ---
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Starting pose – same as your OpMode version
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(-90)));

        // Build all the paths once
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        telemetry.addLine("BlueGoalSidePedro: READY");
        telemetry.update();

        // --- WAIT FOR START ---
        waitForStart();
        if (isStopRequested()) return;

        opmodeTimer.resetTimer();
        setPathState(0);

        // Set the initial tray position immediately.
        setTrayPosition(TRAY_POS_1_SCORE);

        // --- MAIN AUTONOMOUS LOOP ---
        while (opModeIsActive() && !isStopRequested()) {

            // Pedro follower must be updated every loop
            follower.update();

            // Drive the state machine
            pathState = autonomousPathUpdate();

            runIntakeLifterWithColorSensor();

            // Panels/driver telemetry
            panelsTelemetry.addData("Tray Curr", currentTrayPosition);
            panelsTelemetry.addData("Path State", pathState);
            panelsTelemetry.addData("X", follower.getPose().getX());
            panelsTelemetry.addData("Y", follower.getPose().getY());
            panelsTelemetry.addData("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);

            telemetry.update();
        }
    }

    /**
     * Inner class defining all the Pedro paths.
     */
    public static class Paths {
        public Paths(Follower follower) {

        }

    }

    /**
     * State machine for autonomous sequence.
     * Returns current pathState for logging.
     */
    public int autonomousPathUpdate() {

        // Helpful debug info every loop
        telemetry.addData("PathState", pathState);
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());

        switch (pathState) {
            case 0:
                telemetry.addLine("Case " + pathState + ": Start Path1");

                // Set the initial tray position
                setTrayPosition(TRAY_POS_1_SCORE);
                follower.followPath(paths.Path1);
                setPathState(pathState + 1);
                break;

            case 1:
                telemetry.addLine("Case " + pathState + ": Wait for Path1 and camera, then start read AprilTag");

                if ((!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.0)) {

                    telemetry.addLine("Case " + pathState + ": exiting");
                    // Start AprilTag reading after path1 is done
                    tagFSM.start(getRuntime());

                    setPathState(pathState + 1);
                }
                break;

            case 2:
                telemetry.addLine("Case " + pathState + ": Read AprilTag then start Path2");

                tagFSM.update(getRuntime(), true, telemetry);

                if ((tagFSM.isDone()) || pathTimer.getElapsedTimeSeconds() > 2.67) {
                    aprilTagDetections = tagFSM.getDetections();

                    telemetry.addLine("Case " + pathState + ": exiting");
                    follower.followPath(paths.Path2);
                    setPathState(pathState + 1);
                }
                break;

            case 3:
                telemetry.addLine("Case " + pathState + ": Wait for Path2, then shoot artifact");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    telemetry.addLine("Case " + pathState + ": exiting");
                    setPathState(pathState + 1);
                }
                break;

            case 4:
                telemetry.addLine("Case " + pathState + ": start shooting");

                shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), 0.9);

                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    setPathState(pathState + 1);
                }
                break;

            case 5:
                telemetry.addLine("Case " + pathState + ": updateShooting...");
                shootPatternFSM.updateShootPattern(getRuntime());

                if (shootPatternFSM.isShootPatternDone() || pathTimer.getElapsedTimeSeconds() > 10.0) {

                    rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);

                    // now continue with next path
                    follower.followPath(paths.Path3, true);
                    setPathState(pathState + 1);
                }
                break;

            case 6:
                telemetry.addLine("Case " + pathState + ": Wait for Path3, then start Path4");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4.0) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 1p");

                    follower.setMaxPower(0.175); //slow down for pickup

                    setTrayPosition(TRAY_POS_1_INTAKE);
                    follower.followPath(paths.Path4, true);
                    setPathState(pathState + 1);
                }
                break;

            case 7:
                telemetry.addLine("Case " + pathState + ": Wait for Path4");
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4.2) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 2p");

                    setTrayPosition(TRAY_POS_3_INTAKE);
                    follower.followPath(paths.Path5, true);
                    setPathState(pathState + 1);
                }
                break;

            case 8:
                telemetry.addLine("Case " + pathState + ": Wait for Path5");
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.3) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 3g");

                    setTrayPosition(TRAY_POS_2_INTAKE);
                    follower.followPath(paths.Path6, true);
                    setPathState(pathState + 1);
                }
                break;

            case 9:
                telemetry.addLine("Case " + pathState + ": Wait for Path6 to pick up artifact, then start Path7");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    telemetry.addLine("Case " + pathState + ": Moving to shooting position");

                    follower.setMaxPower(0.8); //resume normal speed
                    follower.followPath(paths.Path7, true);
                    setTrayPosition(TRAY_POS_1_SCORE);
                    rubberBands.setPower(0);
                    setPathState(pathState + 1);
                }
                break;

            case 10:
                telemetry.addLine("Case " + pathState + ": Wait for Path7 to get into position, then start Path8");
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    telemetry.addLine("Case " + pathState + ": Shoot the pattern");
                    shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), 0.9); // TODO: adjust power as needed with voltage
                    setPathState(pathState + 1);
                }
                break;

            case 11:
                telemetry.addLine("Case " + pathState + ": Wait for Path6 to finish, then stop");
                shootPatternFSM.updateShootPattern(getRuntime());
                if (shootPatternFSM.isShootPatternDone()) {
                    telemetry.addLine("Case " + pathState + ": Done, setting state -1");
                    rubberBands.setPower(0);
                    setPathState(-1); // done
                }
                break;

            default:
                // -1 or any undefined state: do nothing, stay idle
                telemetry.addLine("Idle state (pathState = " + pathState + ")");
                break;
        }

        return pathState;
    }

    /**
     * Sets the path state and resets its timer.
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}