package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Bert_Auto1-Fwd 30")
public class Bert_Auto1 extends LinearOpMode {
    Bert_Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Bert_Robot(this);


        while (!isStarted()) {
            sleep(100);
            telemetry.update();
        }

        robot.goForward(30);
    }
}