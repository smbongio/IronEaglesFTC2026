package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.Alliance;

@Autonomous(name = "IronEagles Auto RED", group = "Autonomous", preselectTeleOp = "IronEaglesTeleOp2026")
public class IronEaglesAutoRed extends IronEaglesAutoBase {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
