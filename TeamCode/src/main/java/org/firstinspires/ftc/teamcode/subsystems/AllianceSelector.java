package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.interfaces.AllianceInterface;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

public class AllianceSelector implements AllianceInterface {
    private final RobotLogger log = new RobotLogger("ALLIANCE");
    private Color color;
    private boolean locked;

    @Override
    public void choose(Color color) {
        if (locked) {
            log.w("Ignored choose(%s); alliance is locked", color);
            return;
        }
        this.color = color;
        log.i("Alliance set to %s", color);
    }

    @Override
    public void lock() {
        if (color != null && !locked) {
            locked = true;
            log.i("Alliance locked: %s", color);
        }
    }

    @Override
    public boolean isSet() {
        return color != null;
    }

    @Override
    public boolean isLocked() {
        return locked;
    }

    @Override
    public Color getColor() {
        return color;
    }
}
