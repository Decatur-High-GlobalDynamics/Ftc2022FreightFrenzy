package DhsTeamCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Func;

public class TeamRobot
{
    OpMode opMode;
    DcMotor leftDrive, rightDrive;
    BNO055IMU imu;

    public TeamRobot(OpMode _opMode) {
        opMode = _opMode;
        rightDrive= opMode.hardwareMap.dcMotor.get("right_drive");
        leftDrive= opMode.hardwareMap.dcMotor.get("left_drive");

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);

        opMode.telemetry.addData("G1", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        Gamepad gp = opMode.gamepad1;
                        return String.format("%s:%+3.1f,%+3.1f|%s:%+3.1f,%+3.1f|%s%s%s%s|%s%s%s%s|%s %s|t:%3.1f/%3.1f",
                                gp.left_stick_button ? "LJ" : "lj",gp.left_stick_x, gp.left_stick_y,
                                gp.right_stick_button ? "RJ" : "rj", gp.right_stick_x, gp.right_stick_y,
                                gp.a ? "A" : "a", gp.b ? "B" : "b", gp.x ? "X" : "x", gp.y ? "Y" : "y",
                                gp.dpad_up ? "U" : "u", gp.dpad_down ? "D" : "d", gp.dpad_left ? "L" : "l", gp.dpad_right ? "R" : "r",
                                gp.left_bumper ? "LB" : "lb", gp.right_bumper ? "RB" : "rb",
                                gp.left_trigger, gp.right_trigger);
                    }
                });

        opMode.telemetry.addData("Drive", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("Left=%+6.1f %8d |Right=%+6.1f %8d",
                                leftDrive.getPower(),leftDrive.getCurrentPosition(),
                                rightDrive.getPower(), rightDrive.getCurrentPosition());
                    }
                });
    }

    public TeamRobot setLeftPower(double power) {
        leftDrive.setPower(power);
        return this;
    }

    public TeamRobot setRightPower(double power) {
        rightDrive.setPower(-power);
        return this;
    }
}
