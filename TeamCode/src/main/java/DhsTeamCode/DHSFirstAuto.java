package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

@Autonomous(name="DhsFirstAuto-Fwd&Back")
@Disabled
public class DHSFirstAuto extends TeamOpMode {
    @Override
    public void teamRunOpMode() throws InterruptedException {
        robot.setLeftPower(0.5);
        robot.setRightPower(0.5);

        Thread.sleep(10000);

        robot.setLeftPower(-0.5);
        robot.setRightPower(-0.5);

        robot.sleep(10000);

    }
}
