package DhsTeamCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.TetrixMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;

public class Bert_Robot {
    public final double TICKS_PER_INCH = 10000/84.75;

    public final int ARM_BOTTOM_POSITION=0,
                     ARM_LOW_POSITION=1021,
                     ARM_MID_POSITION=1321,
                     ARM_HIGH_POSITION=1621,
                     ARM_LIMIT_POSITION = 1800;

    public final double ARM_DOWN_SPEED_WHEN_RESETTING=-0.10;
    public final double ARM_LIFT_SPEED = 0.30;
    public final double ARM_DOWN_SPEED = -0.20;
    public final Servo leftWhisker;
    public final Servo rightWhisker;
    public final double LEFTWHISKER_STARTPOSITION=0;
    public final double RIGHTWHISKER_STARTPOSITION=1.0;
    public final double RIGHTWHISKER_MAXPOSITION=.2;
    public final double LEFTWHISKER_MAXPOSITION=.6;
    public final double LEFTWHISKER_MIDPOSITION=.3;
    public final double RIGHTWHISKER_MIDPOSITION=.5;









    OpMode opMode;
    BertDcMotor leftDrive, rightDrive;
    BertDcMotor liftMotor, frontDoor;
    Servo backDoor;
    BertDcMotor turn_table;

    long lastLoopTime_ms = System.currentTimeMillis();

    // Gyro variables
    float heading_totalDegreesTurned, desiredHeading_totalDegreesTurned;
    float lastImuReading;

    BNO055IMU imu;
    long initializedTime_ms = System.currentTimeMillis();
    long opModeStarted_ms=-1;
    String status="";
    long statusChangedTime_ms = System.currentTimeMillis();

    public Bert_Robot(OpMode _opMode) {
        opMode = _opMode;
        rightDrive= new BertDcMotor(opMode, "right_drive");
        rightDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive=new BertDcMotor(opMode, "left_drive");

        liftMotor=new BertDcMotor(opMode, "cage_lift");
        liftMotor.setMinimumPosition(0);
        liftMotor.setMaximumPosition(ARM_LIMIT_POSITION);

        turn_table=new BertDcMotor(opMode, "turn_table");
        turn_table.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontDoor=new BertDcMotor(opMode, "front_door");


        backDoor  = opMode.hardwareMap.servo.get("back_door");

        leftWhisker  = opMode.hardwareMap.servo.get("left_whisker");

        rightWhisker  = opMode.hardwareMap.servo.get("right_whisker");

        leftWhisker.setPosition(0.5);

        rightWhisker.setPosition(0.5);

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
        lastImuReading = heading_totalDegreesTurned = _getHeadingFromImu();

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
                                return String.format("Left=%s|Right=%s",
                                        leftDrive.getTelemetryString(),
                                        rightDrive.getTelemetryString());
                            }
                        });

        opMode.telemetry.addData("IMU", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("hdg=%+6.1f | goal=%+6.1f | err=%+3.0f | raw=%+6.1f",
                                heading_totalDegreesTurned, desiredHeading_totalDegreesTurned,
                                desiredHeading_totalDegreesTurned - heading_totalDegreesTurned,
                                lastImuReading);
                    }
                });
        opMode.telemetry.addData("Arm", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("Lift: %s | F: %s |  B: %.2f",
                                liftMotor.getTelemetryString(),
                                frontDoor.getTelemetryString(),
                                backDoor.getPosition());
                    }
                });
        opMode.telemetry.addData("Whiskers", "%s",
                new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("Left: %.2f | Right: %.2f" ,
                                leftWhisker.getPosition(),
                                rightWhisker.getPosition());


                    }
                });

        resetArm();
        sleep(100);
    }

    private float _getHeadingFromImu() {
        return imu.getAngularOrientation().firstAngle;
    }

    public void setLeftPower(double power) {
        leftDrive.setPower(power);
    }

    public void setRightPower(double power) {
        rightDrive.setPower(power);
    }

    public void setDrivePower(double power) {
        setLeftPower(power);
        setRightPower(power);
    }

    public void noteThatOpModeStarted() {
        if ( opModeStarted_ms == -1 )
            opModeStarted_ms = System.currentTimeMillis();
    }

    // Describe what the robot is doing
    public void updateStatus(String statusFormat, Object... formatArgs) {
        String newStatus = String.format(statusFormat, formatArgs);
        if ( !status.equals(newStatus) ) {
            status = newStatus;
            statusChangedTime_ms = System.currentTimeMillis();
        }
        loop();
    }

    public void sleep(long mSec) {
        long start = System.currentTimeMillis();
        long end   = start + mSec;

        while ( System.currentTimeMillis() <= end ) {
            loop();
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException("Interrupted");
            }
        }
        // Make sure we do at least one loop
        loop();
    }

    public void goForward(float inches) throws InterruptedException {
        desiredHeading_totalDegreesTurned = heading_totalDegreesTurned;
        int startPosition=leftDrive.getCurrentPosition();
        long endPosition= Math.round(startPosition+inches*TICKS_PER_INCH);
        setDrivePower(.40);

        while(leftDrive.getCurrentPosition()<endPosition){
            sleep(10);
        }
        setDrivePower(0);
    }

    public void goBackward(float inches) throws InterruptedException {
        desiredHeading_totalDegreesTurned = heading_totalDegreesTurned;
        int startPosition=leftDrive.getCurrentPosition();
        long endPosition= Math.round(startPosition-inches*TICKS_PER_INCH);
        setDrivePower(-.40);

        while(leftDrive.getCurrentPosition()>endPosition){
            sleep(10);
        }
        setDrivePower(0);
    }
    public void turnLeft(float degrees) throws InterruptedException {
        desiredHeading_totalDegreesTurned = heading_totalDegreesTurned;
        double startHeading= heading_totalDegreesTurned;
        double endHeading= startHeading-degrees;
        setLeftPower(-0.30);setRightPower(0.30);

        while(heading_totalDegreesTurned>endHeading){
            sleep(10);
        }
        setDrivePower(0);
    }
    public void turnRight(float degrees) throws InterruptedException {
        desiredHeading_totalDegreesTurned = heading_totalDegreesTurned;
        double startHeading= heading_totalDegreesTurned;
        double endHeading= startHeading+degrees;
        setLeftPower(0.30);setRightPower(-0.30);

        while(heading_totalDegreesTurned<endHeading){
            sleep(10);
        }
        setDrivePower(0);
    }




    private void resetArm() {
        liftMotor.disableLimitChecks();
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(ARM_DOWN_SPEED_WHEN_RESETTING);
        // let the arm get moving
        sleep(100);

        // When to consider the arm-lift motor to have stopped
        final int stoppedThreshold=25;
        while ( liftMotor.speed_perSec > stoppedThreshold ) {
            sleep(100);
        }

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        holdArmPosition();
        liftMotor.enableLimitChecks();
    }


    public void setArmPosition(int position) {
        if ( position < liftMotor.getCurrentPosition() )
            liftMotor.setPower(0.3);
        else
            liftMotor.setPower(0.5);
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void holdArmPosition() {
        if (liftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            setArmPosition(liftMotor.getCurrentPosition());
    }


    private void setArmPower(double power) {
        if ( power==0 ) {
            holdArmPosition();
            return;
        }

        if (liftMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setPower(power);
    }

    public void moveArmUp() {
        setArmPower(ARM_LIFT_SPEED);
    }

    public void moveArmDown() {
        if ( liftMotor.getCurrentPosition()<500 )
            setArmPower(ARM_DOWN_SPEED_WHEN_RESETTING);
        else if (liftMotor.getCurrentPosition() < 1000 )
            setArmPower( (ARM_DOWN_SPEED_WHEN_RESETTING+ARM_DOWN_SPEED)/2);
        else
            setArmPower(ARM_DOWN_SPEED);
    }

    public void backDoorClose() {
        backDoor.setPosition(0.3);
    }

    public void backDoorOpen() {
        backDoor.setPosition(0);
    }

    public void loop() {
        // Update heading
        float imuReading = _getHeadingFromImu();
        float degreesTurned = imuReading - lastImuReading;
        lastImuReading = imuReading;

        // Check for wrapping around (suddenly we will have turned a lot)
        if ( degreesTurned > 180 )
            degreesTurned -= 360;
        else if ( degreesTurned < -180 )
            degreesTurned += 360;

        heading_totalDegreesTurned += degreesTurned;

        leftDrive.loop();
        rightDrive.loop();
        liftMotor.loop();
        frontDoor.loop();
        turn_table.loop();
    }

    public void setTurntablePower(double percent) {
        final double MAX_POWER=1.0;

        turn_table.setPower(MAX_POWER*percent);
        turn_table.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

