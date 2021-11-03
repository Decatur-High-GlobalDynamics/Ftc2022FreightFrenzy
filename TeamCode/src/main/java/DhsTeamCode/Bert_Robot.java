package DhsTeamCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Func;

public class Bert_Robot {
    OpMode opMode;
    DcMotor leftDrive, rightDrive;
    long lastLoopTime_ms = System.currentTimeMillis();
    long leftDrivePosition_previous=0, rightDrivePosition_previous=0;
    long leftDriveSpeed_perSec=0, rightDriveSpeed_perSec=0;
    
    BNO055IMU imu;
    long initializedTime_ms = System.currentTimeMillis();
    long opModeStarted_ms=-1;
    String status="";
    long statusChangedTime_ms = System.currentTimeMillis();

    public Bert_Robot(OpMode _opMode) {
        opMode = _opMode;
        rightDrive= opMode.hardwareMap.dcMotor.get("right_drive");
        rightDrivePosition_previous = rightDrive.getCurrentPosition();
        
        leftDrive= opMode.hardwareMap.dcMotor.get("left_drive");
        leftDrivePosition_previous = leftDrive.getCurrentPosition();

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);

        opMode.telemetry.addData("Robot", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        long now_ms = System.currentTimeMillis();
                        return String.format("t=%3ds/%3ds|%s (%3ds)",
                                (now_ms - initializedTime_ms)/1000,
                                opModeStarted_ms==-1 ? -1 : (now_ms-opModeStarted_ms)/1000,
                                status, (now_ms - statusChangedTime_ms)/1000);
                    }
                });

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
                                return String.format("Left=%+4.1f %5d/sec|Right=%+4.1f %5d/sec",
                                        leftDrive.getPower(), leftDriveSpeed_perSec,
                                        rightDrive.getPower(), rightDriveSpeed_perSec);
                            }
                        });

        opMode.telemetry.addData("IMU", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("hdg first=%+6.2f second=%+6.2f third=%+5.2f",
                                imu.getAngularOrientation().firstAngle,
                                imu.getAngularOrientation().secondAngle,
                                imu.getAngularOrientation().thirdAngle);
                    }
                });
    }

    public Bert_Robot setLeftPower(double power) {
        //Negative power on the left is forward, so we need to reverse it
        leftDrive.setPower(-power);
        return this;
    }

    public Bert_Robot setRightPower(double power) {
        rightDrive.setPower(power);
        return this;
    }

    public void noteThatOpModeStarted() {
        if ( opModeStarted_ms == -1 )
            opModeStarted_ms = System.currentTimeMillis();
    }

    public Bert_Robot updateStatus(String statusFormat, Object... formatArgs) {
        String newStatus = String.format(statusFormat, formatArgs);
        if ( !status.equals(newStatus) ) {
            status = newStatus;
            statusChangedTime_ms = System.currentTimeMillis();
        }

        return this;
    }

    public void loop() {
        // Calculate motor speeds
        long now = System.currentTimeMillis();
        
        // Just return if no time has passed
        if ( now == lastLoopTime_ms )
            return;
        
        long leftDrivePosition = leftDrive.getCurrentPosition();
        long rightDrivePosition= rightDrive.getCurrentPosition();
        
        leftDriveSpeed_perSec = (leftDrivePosition-leftDrivePosition_previous)*1000/(now-lastLoopTime_ms);
        rightDriveSpeed_perSec = (rightDrivePosition-rightDrivePosition_previous)*1000/(now-lastLoopTime_ms);

        leftDrivePosition_previous = leftDrivePosition;
        rightDrivePosition_previous= rightDrivePosition;
    }
}
