package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Bert_Auto1", group="Testing")
public class Dhs2ndAuto extends LinearOpMode {
    Bert_Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Bert_Robot(this);


        while (!isStarted()) {
            sleep(100);
            telemetry.update();
        }

        robot.setArmPosition(robot.ARM_MID_POSITION);
        //when the arm lifts up we are forward 29 inches and the middle tier of the
        //shipping hub is 14 inches away from the edge of the robot
        robot.goForward(14);
        robot.backDoorOpen();
        robot.sleep(2000);
        robot.goBackward(5);
        robot.turnLeft(90);
        robot.goForward(60);
    }
}