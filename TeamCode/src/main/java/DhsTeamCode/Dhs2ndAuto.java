package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Dhs2ndAuto-Freight&Warehouse")
@Disabled
public class Dhs2ndAuto extends TeamOpMode {
    @Override
    public void teamRunOpMode() throws InterruptedException {
        robot.setArmPosition(Robot.ARM_PRESET.MID);
        //when the arm lifts up we are forward 29 inches and the middle tier of the
        //shipping hub is 14 inches away from the edge of the robot
        robot.goForward(14);
        robot.sleep(2000);
        robot.goBackward(5);
        robot.turnLeft(90);
        robot.goForward(60);
    }
}