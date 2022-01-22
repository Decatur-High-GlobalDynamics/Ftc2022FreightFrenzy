package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp1-DriveOnly")
@Disabled
public class TeleOp1 extends TeamOpMode {
    @Override
    public void teamLoop() {
        // Gamepad joysticks return negative numbers when pushed forward,
        // so we invert them here so forward makes the robot go forward
        robot.setLeftPower(-gamepad1.left_stick_y);
        robot.setRightPower(-gamepad1.right_stick_y);

        if ( gamepad1.dpad_up ) {
            robot.armLiftMotor.setPower(.25);
            robot.armLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if ( gamepad1.dpad_down ) {
            robot.armLiftMotor.setPower(-0.1);
            robot.armLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else {
            if ( robot.armLiftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION ) {
                robot.armLiftMotor.setTargetPosition(robot.armLiftMotor.getCurrentPosition());
                robot.armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armLiftMotor.setPower(0.25);
            }
        }
    }
}
