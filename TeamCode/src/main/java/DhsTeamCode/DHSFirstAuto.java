package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

@Autonomous(name="DhsFirstAuto")
@Disabled
public class DHSFirstAuto extends LinearOpMode {
    TeamRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new TeamRobot(this);

        while ( !isStarted() ) {
            sleep(100);
            telemetry.update();
        }

        robot.setLeftPower(0.5);
        robot.setRightPower(0.5);

        Thread.sleep(10000);

        robot.setLeftPower(-0.5);
        robot.setRightPower(-0.5);

        Thread.sleep(10000);

    }
}
