package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="DhsMeasureEncoders")
public class DHSmeasureEncoders extends TeamOpMode {
    @Override
    public void teamRunOpMode() throws InterruptedException {
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
