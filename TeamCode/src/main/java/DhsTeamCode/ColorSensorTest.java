package DhsTeamCode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Test: ColorSensor", group = "Concept")
public class ColorSensorTest extends LinearOpMode {

    ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "revcolor");

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout","id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Colors", String.format("r%d/g%d/b%d", colorSensor.red(), colorSensor.green(), colorSensor.blue()));
                telemetry.update();
            }
        }
    }
}
