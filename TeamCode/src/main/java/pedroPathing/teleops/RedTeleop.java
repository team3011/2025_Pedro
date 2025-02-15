package pedroPathing.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RedTeleop.j")
public class RedTeleop extends JavaCompetitionTeleop{
    @Override
    protected AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
}
