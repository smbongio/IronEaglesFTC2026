package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.interfaces.LedInterface;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

public class NoOpLed implements LedInterface {
    private final RobotLogger log = new RobotLogger("LED");
    private State state = State.OFF;

    @Override
    public void setState(State state) {
        if (this.state != state) {
            log.i("LED %s -> %s (no hardware; Hub telemetry mirrors this)", this.state, state);
            this.state = state;
        }
    }

    @Override
    public State getState() {
        return state;
    }
}
