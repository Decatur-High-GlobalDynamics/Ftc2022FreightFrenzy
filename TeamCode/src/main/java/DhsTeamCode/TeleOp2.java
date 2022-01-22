package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="TeleOp2", group="Competition")
public class TeleOp2 extends TeamOpMode {
    boolean lastGrabStatus=false;

    @Override
    public void teamLoop() {
        // Two modes for driving:
        //  1. Gamepad2: go forward slowly when right joystick is moved
        //  2. Gamepad1: Normal two-stick driving

        if (gamepad1.right_bumper)
            robot.tiltprotectionenabled=false;
        else
            robot.tiltprotectionenabled=true;

        if ( gamepad2.left_stick_y < -0.1 ) {
            // go forward slowly
            robot.setDrivePower(0.2);
        } else if ( gamepad2.left_stick_y > 0.1 ) {
            robot.setDrivePower(-0.2);
        } else {
            // Gamepad joysticks return negative numbers when pushed forward,
            // so we invert them here so forward makes the robot go forward
            if (gamepad1.left_bumper) {
                robot.setLeftPower(-gamepad1.left_stick_y * 0.5);
                robot.setRightPower(-gamepad1.right_stick_y * 0.5);
            } else {
                robot.setLeftPower(-gamepad1.left_stick_y);
                robot.setRightPower(-gamepad1.right_stick_y);
            }
        }

        // Dpad controls arm:
        //    Up/Down: Change between ARM preset positions
        //    Left/Right: move arm down/up
        if (gamepad2.dpad_up) {
            robot.moveArm_presetUp();
        } else if (gamepad2.dpad_down) {
            robot.moveArm_presetDown();
        } else if ( gamepad2.dpad_right) {
            robot.moveArm_up();
        } else if ( gamepad2.dpad_left ) {
            robot.moveArm_down();
        } else {
            robot.holdArmAtPosition();
        }

        // Grabbers are controlled by the right_stick on Gamepad2:
        //   Default: Grabbing
        //   x: 0 grabs, everything else releases
        //   y: 0 grabs and then it gradually releases
        if ( Math.abs(gamepad2.right_stick_x) > 0.25 )
            robot.grabbers_release();
        else if ( Math.abs(gamepad2.right_stick_y) > 0.1)
            robot.grabber_setPosition(gamepad2.right_stick_y);
        else
            robot.grabbers_grab();

        double turntablePower = gamepad2.right_trigger;

        // GP2 LeftBumper reverses the direction
        if (gamepad2.left_bumper)
            turntablePower = -1 * turntablePower;

        robot.setTurntablePower(turntablePower);
    }

    }
