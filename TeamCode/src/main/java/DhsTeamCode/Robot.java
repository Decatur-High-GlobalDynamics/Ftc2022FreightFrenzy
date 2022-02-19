package DhsTeamCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;

import androidx.annotation.NonNull;

public class Robot {

    private static final int MAX_ELEVATOR_POSITION = 3500;
    private static final int MIN_ELEVATOR_POSITION = 20;
    private static final double SLOW_POWER_FACTOR = 0.5;

    public final double TICKS_PER_INCH = 5000/103.0;
    public double defaultElevatorPower = 0.1;
    public final double MAX_CUP_TURN_SERVO_POS = 1.0; // Default position
    public final double MIN_CUP_TURN_SERVO_POS = 0.45;
    public final double MAX_CUP_DUMP_SERVO_POS = .80; // Default position
    public final double MIN_CUP_DUMP_SERVO_POS = 0.12;

    public final TeamServo cupTurnServo;
    public final TeamServo cupDumpServo;

    public boolean slowDriving = false;

    TeamOpMode opMode;
    TeamDcMotor leftDrive, rightDrive;
    TeamDcMotor spintake;
    TeamDcMotor elevator;
    TeamDcMotor turn_table;

    DigitalChannel cup_touch;

    // Gyro variables
    float heading_totalDegreesTurned, desiredHeading_totalDegreesTurned;
    float lastImuReading1, lastImuReading2, lastImuReading3;

    BNO055IMU imu;
    long initializedTime_ms = System.currentTimeMillis();
    long opModeStarted_ms=-1;
    String status="";
    long statusChangedTime_ms = System.currentTimeMillis();
    private long lastLoopStartTime_ms = System.currentTimeMillis();

    public Robot(TeamOpMode _opMode) throws InterruptedException {
        opMode = _opMode;
        updateStatus("initializing");

        rightDrive = new TeamDcMotor(this, opMode, "right_drive");
        rightDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive = new TeamDcMotor(this, opMode, "left_drive");

        spintake = new TeamDcMotor(this, opMode, "spintake");
        elevator = new TeamDcMotor(this, opMode, "elevator");
        elevator.setMaximumSafetyPosition(MAX_ELEVATOR_POSITION);
        elevator.setMinimumSafetyPosition(MIN_ELEVATOR_POSITION);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cupTurnServo = new TeamServo(this, opMode, "cup_turn_servo");
        cupTurnServo.setMaximumSafetyPosition(MAX_CUP_TURN_SERVO_POS);
        cupTurnServo.setMinimumSafetyPosition(MIN_CUP_TURN_SERVO_POS);
        cupDumpServo = new TeamServo(this, opMode, "cup_dump_servo");
        cupDumpServo.setMaximumSafetyPosition(MAX_CUP_DUMP_SERVO_POS);
        cupDumpServo.setMinimumSafetyPosition(MIN_CUP_DUMP_SERVO_POS);

        turn_table = new TeamDcMotor(this, opMode, "turn_table");
        turn_table.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cup_touch = opMode.hardwareMap.digitalChannel.get("cup_touch");
        cup_touch.setMode(DigitalChannel.Mode.INPUT);

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
        lastImuReading1 = heading_totalDegreesTurned = _getHeadingFromImu();

        opMode.telemetry.addData("R", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        long now_ms = System.currentTimeMillis();
                        return String.format("t_init=%3ds t_run=%3ds %4d loops/s|%s (%3ds)",
                                (now_ms - initializedTime_ms)/1000,
                                opModeStarted_ms==-1 ? -1 : (now_ms-opModeStarted_ms)/1000,
                                opMode.durationOfLastLoop_ms == 0 ? 1000 : 1000/opMode.durationOfLastLoop_ms,
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

        opMode.telemetry.addData("G2", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        Gamepad gp = opMode.gamepad2;
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
                                return String.format("Left=%s|Right=%s|Slow=%s",
                                        leftDrive.getTelemetryString(),
                                        rightDrive.getTelemetryString(),
                                        slowDriving ? "Slow" : "Normal");
                            }
                        });

        opMode.telemetry.addData("IMU", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("hdg=%+.1f | goal=%+.1f | err=%+.1f | raw=%+.1f/%+.1f/%+.1f",
                                heading_totalDegreesTurned, desiredHeading_totalDegreesTurned,
                                desiredHeading_totalDegreesTurned - heading_totalDegreesTurned,
                                lastImuReading1, lastImuReading2, lastImuReading3);
                    }
                });

        opMode.telemetry.addData("Turntable", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return turn_table.getTelemetryString();
                    }
                });

        opMode.telemetry.addData("Spintake", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return spintake.getTelemetryString();
                    }
                });

        opMode.telemetry.addData("Elevator", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("%s | defPow = %.2f | cupTouch %s", elevator.getTelemetryString(), defaultElevatorPower, cup_touch.getState() ? "Pressed" : "Unpressed");
                    }
                });

        opMode.telemetry.addData("Cup Servos", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("turn: %s | dump: %s", cupTurnServo.getServoTelemetryString(), cupDumpServo.getServoTelemetryString());
                    }
                });
        sleep(100);
        updateStatus("initialized");

    }

    private float _getHeadingFromImu() {
        return imu.getAngularOrientation().firstAngle;
    }

    private float _getAngle2FromImu() {
        return imu.getAngularOrientation().secondAngle;
    }
    private float _getAngle3FromImu() {
        return imu.getAngularOrientation().thirdAngle;
    }
    /**
     * Set the left power, based on whether robot is reversed
     * @param power
     */
    public void setLeftPower(double power) {
        if (slowDriving)
        {
            power = power * SLOW_POWER_FACTOR;
        }
        leftDrive.setPower(power);
    }

    /**
     * set the right power, based on whether robot is reversed
     * @param power
     */
    public void setRightPower(double power) {
        if (slowDriving)
        {
            power = power * SLOW_POWER_FACTOR;
        }
        rightDrive.setPower(power);
    }

    public void setDrivePower(double power) {
        setLeftPower(power);
        setRightPower(power);
    }

    public void noteThatOpModeStarted() throws InterruptedException {
        opModeStarted_ms = System.currentTimeMillis();
    }

    // Describe what the robot is doing
    public void updateStatus(String statusFormat, Object... formatArgs) {
        String newStatus = String.format(statusFormat, formatArgs);
        if ( !status.equals(newStatus) ) {
            status = newStatus;
            statusChangedTime_ms = System.currentTimeMillis();
        }
    }

    public void sleep(long mSec) throws InterruptedException {
        long start = System.currentTimeMillis();
        long end   = start + mSec;

        while ( opMode.opModeIsActive() && System.currentTimeMillis() <= end ) {
            loop();
            Thread.sleep(5);
        }
        // Make sure we do at least one loop
        loop();
    }

    public void goForward(float inches) throws InterruptedException {
        desiredHeading_totalDegreesTurned = heading_totalDegreesTurned;
        int startPosition=leftDrive.getCurrentPosition();
        long endPosition= Math.round(startPosition+inches*TICKS_PER_INCH);
        setDrivePower(0.70);

        while(leftDrive.getCurrentPosition()<endPosition && !opMode.isStopRequested()){
            sleep(10);
        }
        setDrivePower(0);
    }

    public void goBackward(float inches) throws InterruptedException {
        desiredHeading_totalDegreesTurned = heading_totalDegreesTurned;
        int startPosition=leftDrive.getCurrentPosition();
        long endPosition= Math.round(startPosition-inches*TICKS_PER_INCH);
        setDrivePower(-0.70);

        while(leftDrive.getCurrentPosition()>endPosition && !opMode.isStopRequested()){
            sleep(10);
        }
        setDrivePower(0);
    }
    public void turnLeft(float degrees) throws InterruptedException {
        desiredHeading_totalDegreesTurned = heading_totalDegreesTurned;
        double startHeading= heading_totalDegreesTurned;
        double endHeading= startHeading-degrees;
        setLeftPower(-0.30);setRightPower(0.30);

        while(heading_totalDegreesTurned>endHeading && !opMode.isStopRequested()){
            sleep(10);
        }
        setDrivePower(0);
    }
    public void turnRight(float degrees) throws InterruptedException {
        desiredHeading_totalDegreesTurned = heading_totalDegreesTurned;
        double startHeading= heading_totalDegreesTurned;
        double endHeading= startHeading+degrees;
        setLeftPower(0.30);setRightPower(-0.30);

        while(heading_totalDegreesTurned<endHeading && !opMode.isStopRequested()){
            sleep(10);
        }
        setDrivePower(0);
    }

    public void loop() {
        long now_ms = System.currentTimeMillis();
        lastLoopStartTime_ms = now_ms;

        // Update heading
        float imuReading = _getHeadingFromImu();
        float degreesTurned = imuReading - lastImuReading1;
        lastImuReading1 = imuReading;
        lastImuReading2 = _getAngle2FromImu();
        lastImuReading3 = _getAngle3FromImu();

        // Elevator zero-setting
        /*
        if (elevator.getPower() < 0 && cup_touch.getState())
            elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */

        // Check for wrapping around (suddenly we will have turned a lot)
        if ( degreesTurned > 180 )
            degreesTurned -= 360;
        else if ( degreesTurned < -180 )
            degreesTurned += 360;

        heading_totalDegreesTurned += degreesTurned;

        leftDrive.loop();
        rightDrive.loop();
        turn_table.loop();
        elevator.loop();
        spintake.loop();

        opMode.telemetry.update();
    }

    public void setTurntablePower(double percent) {
        final double MAX_POWER = 1.0;

        turn_table.setPower(MAX_POWER * percent);
        turn_table.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSpintakePower(double percent) {
        final double MAX_POWER = 1.0;

        spintake.setPower(MAX_POWER * percent);
        spintake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setElevatorPower(double percent) {
        final double MAX_POWER = 1.0;

        elevator.setPower(MAX_POWER * percent);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setCupTurnServoPos(double position) {
        cupTurnServo.setPosition(position);
    }

    public void setCupDumpServoPos(double position) {
        cupDumpServo.setPosition(position);
    }

    /**
     * Sets the position of the grabber. 1 is fully out, 0 is fully in/grabbing.
     * @param position
     */
}

