package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp2", group="Competition")
public class TeleOp2 extends TeamOpMode {
    @Override
    public void teamLoop() {
        // Two modes for driving:

        // Gamepad joysticks return negative numbers when pushed forward,
        // so we invert them here so forward makes the robot go forward
        if (gamepad1.left_bumper) {
            robot.setLeftPower(-gamepad1.left_stick_y * 0.5);
            robot.setRightPower(-gamepad1.right_stick_y * 0.5);
        } else {
            robot.setLeftPower(-gamepad1.left_stick_y);
            robot.setRightPower(-gamepad1.right_stick_y);
        }

        double turntablePower = gamepad1.left_trigger;

        // GP2 LeftBumper reverses the direction
        if (gamepad1.b)
            turntablePower *= -1;

        robot.setTurntablePower(turntablePower);

        double spintakePower = gamepad1.right_trigger;

        if (gamepad1.a)
            spintakePower *= -1;

        robot.setSpintakePower(spintakePower);

        if (gamepad2.y)
            robot.defaultElevatorPower = Range.clip(robot.defaultElevatorPower + 0.05 , 0 , 1);
        if (gamepad2.a)
            robot.defaultElevatorPower = Range.clip(robot.defaultElevatorPower - 0.05 , 0 , 1);
        if (gamepad2.x)
            robot.defaultElevatorPower = 0.1;

        double elevatorPower = Math.max(robot.defaultElevatorPower, gamepad2.left_trigger);

        if (gamepad2.dpad_up)
            robot.setElevatorPower(elevatorPower);
        else if (gamepad2.dpad_down)
            robot.setElevatorPower(-elevatorPower);
        else
            robot.setElevatorPower(0);

        if (gamepad2.left_bumper) {
            robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (gamepad2.left_stick_x > 0 && gamepad2.left_stick_button)
            robot.setCupTurnServoPos(Range.clip(robot.cupTurnServo.getPosition() + 0.05 , 0 , 1));
        else if (gamepad2.left_stick_x < 0 && gamepad2.left_stick_button)
            robot.setCupTurnServoPos(Range.clip(robot.cupTurnServo.getPosition() - 0.05 , 0 , 1));

        if (gamepad2.right_stick_y > 0 && gamepad2.right_stick_button)
            robot.setCupDumpServoPos(Range.clip(robot.cupDumpServo.getPosition() + 0.05 , 0 , 1));
        else if (gamepad2.right_stick_y < 0 && gamepad2.right_stick_button)
            robot.setCupDumpServoPos(Range.clip(robot.cupDumpServo.getPosition() - 0.05 , 0 , 1));
    }

    }
