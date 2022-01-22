package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DhsFirstTeleOpX")
@Disabled
public class DhsFirstTeleOp extends TeamOpMode {
    @Override
    public void teamLoop() {
        // The joysticks report negative values when pressed forward
        // and since we want forward on the joysticks to make the robot
        // go forward, we invert the +/- of the sticks
        robot.setLeftPower(-gamepad1.left_stick_y);
        robot.setRightPower(-gamepad1.right_stick_y);
    }
}