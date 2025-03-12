package drive.writtenCode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;


@TeleOp(name="!AutoTuner", group="Linear OpMode")
public class AutoTuner extends LinearOpMode {


    public static int cycle = 1;
    public static double XRate=1.4;
    public static int SlidesRate=2000;

    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        waitForStart();
        while (opModeIsActive() && !isStopRequested())
        {
            if(isStopRequested()) return;
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left)
            {
                PersistentData.incrementsX[cycle]=PersistentData.incrementsX[cycle]-1;
            }
            if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right)
            {
                PersistentData.incrementsX[cycle]=PersistentData.incrementsX[cycle]+1;
            }

            if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down)
            {
                PersistentData.incrementsSlides[cycle]=PersistentData.incrementsSlides[cycle]-1;
            }
            if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up)
            {
                PersistentData.incrementsSlides[cycle]=PersistentData.incrementsSlides[cycle]+1;
            }
            if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper)
            {
                PersistentData.rotatePosition[cycle] = PersistentData.rotatePosition[cycle]+1;
            }
            if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper)
            {
                PersistentData.rotatePosition[cycle] = PersistentData.rotatePosition[cycle]-1;
            }
            PersistentData.subX[cycle] = 76-PersistentData.incrementsX[cycle]*XRate;
            PersistentData.slidesPosition[cycle] = 0 + PersistentData.incrementsSlides[cycle]*SlidesRate;
            if(currentGamepad1.a && !previousGamepad1.a && cycle<4)
            {
                cycle++;
            }
            if(currentGamepad1.b && !previousGamepad1.b && cycle>1)
            {
                cycle--;
            }
            telemetry.addData("Cycle: ", cycle);
            telemetry.addData("Increments X: ", Arrays.toString(
                    Arrays.copyOfRange(PersistentData.incrementsX, 1, PersistentData.incrementsX.length)
            ));

            telemetry.addData("Increments Slides: ", Arrays.toString(
                    Arrays.copyOfRange(PersistentData.incrementsSlides, 1, PersistentData.incrementsSlides.length)
            ));
            telemetry.addData("Rotate: ", Arrays.toString(
                    Arrays.copyOfRange(PersistentData.rotatePosition, 1, PersistentData.rotatePosition.length)
            ));
            telemetry.addData("SubX: ", Arrays.toString(
                    Arrays.stream(PersistentData.subX)
                            .mapToObj(val -> String.format("%.2f", val)) // Format to 2 decimals
                            .toArray()
            ));
            telemetry.addData("SlidesPos: ", Arrays.toString(PersistentData.slidesPosition));
            telemetry.update();
        }
    }
}
