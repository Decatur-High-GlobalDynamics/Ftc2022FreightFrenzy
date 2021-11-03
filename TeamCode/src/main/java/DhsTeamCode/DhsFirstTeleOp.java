package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DhsFirstTeleOpX", group="Ftc3543")
public class DhsFirstTeleOp extends OpMode {
    TeamRobot robot;

    @Override
    public void init() {
        robot = new TeamRobot(this);
    }

    @Override
    public void loop() {
        // The joysticks report negative values when pressed forward
        // and since we want forward on the joysticks to make the robot
        // go forward, we invert the +/- of the sticks
        robot.setLeftPower(-gamepad1.left_stick_y);
        robot.setRightPower(-gamepad1.right_stick_y);
    }
}