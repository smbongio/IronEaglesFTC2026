package org.firstinspires.ftc.teamcode.interfaces;

public interface LedInterface {
    enum State { OFF, AUTO_FIRE_AVAILABLE }

    void setState(State state);

    State getState();
}
