package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto1-Fwd 30")
public class Auto1 extends TeamOpMode {
    @Override
    public void teamRunOpMode() throws InterruptedException {
        robot.goForward(30);
    }
}