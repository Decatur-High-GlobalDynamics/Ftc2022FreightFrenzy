package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp2", group="Competition")
public class TeleOp2 extends TeamOpMode {
    private Long slowDrivingChanged_ms = null;

    @Override
    public void teamLoop() {
        // Two modes for driving:
        // slowDriving
        if (gamepad1.left_stick_button)
        {
            if (slowDrivingChanged_ms != null && System.currentTimeMillis() - slowDrivingChanged_ms < 100)
            {
                // ignoring button for 100ms
            }
            else
            {
                robot.slowDriving = !robot.slowDriving;
                slowDrivingChanged_ms = System.currentTimeMillis();
            }
        }

        // Gamepad joysticks return negative numbers when pushed forward,
        // so we invert them here so forward makes the robot go forward
        robot.setLeftPower(-gamepad1.left_stick_y);
        robot.setRightPower(-gamepad1.right_stick_y);

        double turntablePower = gamepad1.left_trigger;

        // GP1 LeftBumper reverses direction of turntable
        if (gamepad1.b)
            turntablePower *= -1;

        robot.setTurntablePower(turntablePower);

        double spintakePower = gamepad1.right_trigger;

        // GP1 RightBumper reverses direction of spintake
        if (gamepad1.right_bumper)
            spintakePower *= -1;

        robot.setSpintakePower(spintakePower);

        // Driver #2 controls
        /*
        if (gamepad2.y)
            robot.defaultElevatorPower = Range.clip(robot.defaultElevatorPower + 0.05 , 0 , 1);
        if (gamepad2.a)
            robot.defaultElevatorPower = Range.clip(robot.defaultElevatorPower - 0.05 , 0 , 1);
        if (gamepad2.x)
            robot.defaultElevatorPower = 0.1;
        */

        double elevatorPower = -gamepad2.left_stick_y;
        if (gamepad2.left_stick_button)
            robot.elevator.disableLimitChecks();
        else
            robot.elevator.enableLimitChecks();

        robot.setElevatorPower(0.4 * elevatorPower);

        if (gamepad2.left_bumper) {
            robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Cup dumping
        if (gamepad2.y) {
            robot.setCupTurnServoPos(robot.MAX_CUP_TURN_SERVO_POS);
            robot.setCupDumpServoPos(robot.MIN_CUP_DUMP_SERVO_POS);
        }
        else if (gamepad2.b) {
            robot.setCupTurnServoPos(robot.MIN_CUP_TURN_SERVO_POS);
            robot.setCupDumpServoPos(robot.MIN_CUP_DUMP_SERVO_POS);
        }
        else
        {
            robot.setCupTurnServoPos(robot.MAX_CUP_TURN_SERVO_POS);
            robot.setCupDumpServoPos(robot.MAX_CUP_DUMP_SERVO_POS);
        }
    }

    }
