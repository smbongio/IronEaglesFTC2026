package org.firstinspires.ftc.teamcode.interfaces;

public interface AutoFireControllerInterface {
    enum State { IDLE, MEASURING, MOVING, SPOOLING, FEEDING }

    void start();

    void abort();

    void update();

    State getState();

    boolean isActive();
}
