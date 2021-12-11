package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;

@TeleOp(name="DhsSingleJoystick", group="Testing")
public class DhsTeleop_SingleJoystick extends OpMode {
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

        double leftpower, rightpower;

        // If the right stick is pushed, then use it to spin,
        // otherwise, use the left stick x & y for single-joystick driving
        if ( Math.abs(gamepad1.right_stick_x) >= 0.1) {
            double spinPower = gamepad1.right_stick_x;
            if ( spinPower < 0 ) {
                // negative spinPower ==> turn left
                leftpower = spinPower; // backwards because spinPower is negative
                rightpower= -1.0*spinPower; // forward because -spinPower is positive
            } else {
                // positive spinPower ==> turn right
                leftpower = spinPower; // forward because spinPower is positive
                rightpower= -1.0 * spinPower; // backwards because -spinPower is negative
            }

        } else {
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            // Power is how far joystick is from the center
            double power = Math.sqrt(x * x + y * y);

            // Split joystick into 4 quadrants:
            //    X is <0 (Left)    or >0 (Right)
            //    Y is <0 (Forward) or >0 (Back)
            if (x >= 0 && y <= 0) {
                //q1... forward and to the right
                leftpower = power;
                rightpower = -power * y;

            } else if (x <= 0 && y <= 0) {
                //q2... forward and to the left
                leftpower = -power * y;
                rightpower = power;

            } else if (x <= 0 && y <= 0) {
                //q3... backwards and to the left
                leftpower = power;
                rightpower = -power * y;
            } else {
                //q4... backwards and to the right
                leftpower = -power;
                rightpower = -power * y;
            }
        }

        if (gamepad1.left_bumper){
            robot.setLeftPower(leftpower*0.5);
            robot.setRightPower(rightpower*0.5);

        } else {
            robot.setLeftPower(leftpower);
            robot.setRightPower(rightpower);
        }


        if ( gamepad2.dpad_up ) {
            robot.moveArmUp();
        }
        else if ( gamepad2.dpad_down ) {
            robot.moveArmDown();
        }
        else {
            robot.holdArmPosition();
        }


        // Open back door when left bumper is pressed
        if (gamepad2.left_trigger > 0.1)
            robot.backDoorOpen();
        else
            robot.backDoorClose();

        if (gamepad2.b)
            // Open front door
            robot.frontDoor.setPower(0.1);
        else if (gamepad2.a)
            // Close front door
            robot.frontDoor.setPower(-0.25);
        else
            robot.frontDoor.setPower(0);

        // Only turned in one direction
        //robot.setTurntablePower(gamepad2.right_trigger);

        double turntablePower = gamepad2.right_trigger;

        // GP2 LeftBumper reverses the direction
        if (gamepad2.left_bumper)
            turntablePower = -1*turntablePower;

        robot.setTurntablePower(turntablePower);
    }

}
