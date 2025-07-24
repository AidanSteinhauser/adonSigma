package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Testing", group = "Adon")
public class Testing extends OpMode {

    @Override
    public void init() {
        // Initialization if needed
    }

    @Override
    public void loop() {
        double adjustableValue = ConfigFile.adjustableValue;

        telemetry.addData("Adjustable Value", adjustableValue);
        telemetry.update();
    }
}