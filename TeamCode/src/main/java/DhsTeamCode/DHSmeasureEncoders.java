package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="DhsmeasureEncoders")
public class DHSmeasureEncoders extends LinearOpMode {
    TeamRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new TeamRobot(this);

        while ( !isStarted() ) {
            sleep(100);
            telemetry.update();
        }

        int startPosition=robot.rightDrive.getCurrentPosition();
        int endPosition= startPosition+5000;
        robot.setLeftPower(0.3);
        robot.setRightPower(0.3);

        while (!isStopRequested() && robot.rightDrive.getCurrentPosition() <endPosition) {
            sleep(100);
            telemetry.update();
        }

        robot.setLeftPower(0);
        robot.setRightPower(0);

        while (!isStopRequested()) {
            sleep(100);
            telemetry.update();
        }

    }
}
