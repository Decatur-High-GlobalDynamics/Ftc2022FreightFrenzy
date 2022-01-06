package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Auto2")
public class Auto2 extends LinearOpMode {
    Bert_Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Bert_Robot(this);


        while (!isStarted()) {
            sleep(100);
            robot.loop();
        }

        robot.setArmPosition(robot.ARM_HIGH_POSITION);
        robot.goForward(21);
        robot.backDoorOpen();
        robot.sleep(1000);
        robot.goBackward(10);
        robot.setArmPosition(robot.ARM_BOTTOM_POSITION);
        robot.goBackward(11);
        robot.setLeftPower(.80);
        robot.sleep(1000);
        robot.setRightPower(.60);
        robot.setLeftPower(.65);




    }
}