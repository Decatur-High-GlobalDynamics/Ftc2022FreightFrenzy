package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto2-Freight&Carousel")
public class Auto2 extends TeamOpMode {
    @Override
    public void teamRunOpMode() throws InterruptedException {
        robot.goForward(21);
        robot.sleep(1000);
        robot.goBackward(10);
        robot.grabbers_release();
        robot.setArmPosition(Robot.ARM_PRESET.GRAB);
        robot.goBackward(11);
        robot.turnRight(90);
        robot.setLeftPower(.80);
        robot.sleep(1000);
        robot.setRightPower(.60);
    }
}