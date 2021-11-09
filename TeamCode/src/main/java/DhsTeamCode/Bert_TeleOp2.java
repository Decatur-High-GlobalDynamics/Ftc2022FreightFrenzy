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

        // Gamepad joysticks return negative numbers when pushed forward,
        // so we invert them here so forward makes the robot go forward
        if (gamepad1.left_bumper){
            robot.setLeftPower(-gamepad1.left_stick_y*0.5);
            robot.setRightPower(-gamepad1.right_stick_y*0.5);

        } else {
            robot.setLeftPower(-gamepad1.left_stick_y);
            robot.setRightPower(-gamepad1.right_stick_y);
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
    }
}
