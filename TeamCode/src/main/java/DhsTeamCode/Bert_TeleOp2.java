package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Bert_TeleOp2", group="Testing")
public class Bert_TeleOp2 extends OpMode {
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

        // Two modes for driving:
        //  1. Gamepad2: go forward slowly when right joystick is moved
        //  2. Gamepad1: Normal two-stick driving

        if (gamepad1.right_bumper)
            robot.tiltprotectionenabled=false;
        else
            robot.tiltprotectionenabled=true;
        
        if ( gamepad2.right_stick_y < -0.1 ) {
            // go forward slowly
            robot.setDrivePower(0.2);
        } else if ( gamepad2.right_stick_y > 0.1 ) {
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


        if (gamepad2.dpad_up) {
            robot.moveArmUp();
            robot.closeFrontDoor();
        } else if (gamepad2.dpad_down) {
            robot.moveArmDown();
            robot.openFrontDoor();
        } else {
            robot.holdArmPosition();
        }


        // Open back door when left bumper is pressed
        if (gamepad2.left_trigger > 0.1)
            robot.backDoorOpen();
        else
            robot.backDoorClose();

        if (gamepad2.b)
            // Open front door
            robot.openFrontDoor();
        else if (gamepad2.a) {
            // Move whiskers out of the way and Close front door
            robot.moveBothWhiskersFullyOut();
            robot.closeFrontDoor();
        }
        if (!gamepad2.dpad_down&&!gamepad2.dpad_up&&!gamepad2.a&&!gamepad2.b)

            robot.holdFrontDoor();

        // Only turned in one direction
        //robot.setTurntablePower(gamepad2.right_trigger);

        double turntablePower = gamepad2.right_trigger;

        // GP2 LeftBumper reverses the direction
        if (gamepad2.left_bumper)
            turntablePower = -1 * turntablePower;

        robot.setTurntablePower(turntablePower);


        if (gamepad2.left_stick_x > 0.1 || gamepad2.left_stick_y < -0.1)
            // DOwn or right close whiskers
            robot.moveBothWhiskersIn();
        else
            robot.moveBothWhiskersOut();


        if (gamepad2.left_stick_button)
            robot.moveBothWhiskersFullyOut();
    }

    }
