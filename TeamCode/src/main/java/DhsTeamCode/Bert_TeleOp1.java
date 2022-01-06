package DhsTeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Bert_TeleOp1")
@Disabled
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

        if ( gamepad1.dpad_up ) {
            robot.liftMotor.setPower(.25);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if ( gamepad1.dpad_down ) {
            robot.liftMotor.setPower(-0.1);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else {
            if ( robot.liftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION ) {
                robot.liftMotor.setTargetPosition(robot.liftMotor.getCurrentPosition());
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(0.25);
            }
        }


        // Open back door when left bumper is pressed
        if (gamepad1.left_trigger > 0.1)
            robot.backDoorOpen();
        else
            robot.backDoorClose();

        if (gamepad1.b)
            // Open front door
            robot.frontDoor.setPower(0.1);
        else if (gamepad1.a)
            // Close front door
            robot.frontDoor.setPower(-0.25);
        else
            robot.frontDoor.setPower(0);
    }
}
