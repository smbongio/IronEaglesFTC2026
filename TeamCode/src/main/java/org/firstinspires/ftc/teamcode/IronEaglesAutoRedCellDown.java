package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.Alliance;

@Autonomous(name = "Red Cell Down", group = "Autonomous", preselectTeleOp = "IronEaglesTeleOp2026")
public class IronEaglesAutoRedCellDown extends IronEaglesAutoBase {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }

    @Override
    protected boolean cellUp() {
        return false;
    }
}
