package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.Alliance;

@Autonomous(name = "Blue Cell Down", group = "Autonomous", preselectTeleOp = "IronEaglesTeleOp2026")
public class IronEaglesAutoBlueCellDown extends IronEaglesAutoBase {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    protected boolean cellUp() {
        return false;
    }
}
