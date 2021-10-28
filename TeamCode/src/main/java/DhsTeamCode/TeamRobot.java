package DhsTeamCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    }

    public TeamRobot setLeftPower(double power) {
        //Negative power on the left is forward
        leftDrive.setPower(-power);
        return this;
    }

    public TeamRobot setRightPower(double power) {
        rightDrive.setPower(power);
        return this;
    }
}
