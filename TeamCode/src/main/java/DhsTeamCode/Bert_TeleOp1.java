package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Bert_TeleOp1", group="Testing")
public class Bert_TeleOp1 extends OpMode {
    Bert_Robot robot;

    @Override
    public void init() {
        robot = new Bert_Robot(this);
        robot.updateStatus("initialized");
    }

    @Override
    public void loop() {
        robot.noteThatOpModeStarted();
        robot.updateStatus("looping");

        // Gamepad joysticks return negative numbers when pushed forward,
        // so we invert them here so forward makes the robot go forward
        robot.setLeftPower(-gamepad1.left_stick_y);
        robot.setRightPower(-gamepad1.right_stick_y);
    }
}
