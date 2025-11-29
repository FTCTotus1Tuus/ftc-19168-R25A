package org.firstinspires.ftc.teamcode.team.fsm;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoIncrementalFSM {
    private Servo servo;
    private double startPos, endPos, duration;
    private double startTime;
    private boolean running = false;

    public void start(Servo servo, double endPos, double startPos, double duration, double currentTime) {
        this.servo = servo;
        this.startPos = startPos;
        this.endPos = endPos;
        this.duration = duration;
        this.startTime = currentTime;
        this.running = true;
    }

    public void update(double currentTime) {
        if (!running) return;
        double elapsed = currentTime - startTime;
        if (elapsed >= duration) {
            servo.setPosition(endPos);
            running = false;
            return;
        }
        double currentPos = startPos + (endPos - startPos) * (elapsed / duration);
        servo.setPosition(currentPos);
    }

    public boolean isRunning() {
        return running;
    }
}
