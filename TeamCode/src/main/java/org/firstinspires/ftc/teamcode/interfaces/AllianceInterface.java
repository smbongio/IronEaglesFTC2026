package org.firstinspires.ftc.teamcode.interfaces;

public interface AllianceInterface {
    enum Color { RED, BLUE }

    void choose(Color color);

    void lock();

    boolean isSet();

    boolean isLocked();

    Color getColor();
}
