package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Forward 30in Test")
public class Auto_Forward30in extends TeamOpMode {
    @Override
    public void teamRunOpMode() throws InterruptedException {
        double startPosition = robot.rightDrive.getCurrentPosition();
        double endPosition = startPosition + robot.TICKS_PER_INCH * 30;
        robot.setLeftPower(0.3);
        robot.setRightPower(0.3);

        while (!isStopRequested() && robot.rightDrive.getCurrentPosition() < endPosition) {
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
