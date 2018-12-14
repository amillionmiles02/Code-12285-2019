package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="CompOneTwo", group="Linear Opmode")
public class CompOneTwo extends LinearOpMode {
    HardwareCompOne          Julieo  = new HardwareCompOne();
    ElapsedTime                 runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        Julieo.init(hardwareMap);

        telemetry.addData("Status", "Ready Man");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftGo;
            double rightGo;
            double liftPower = -gamepad2.left_stick_y;
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            boolean backPressed = gamepad1.x;
            boolean driveLastPass = false;
            boolean driveToggle = false;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.
            liftPower = Range.clip(liftPower, -.6, .6);
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

         //   if (backPressed && driveToggle == false) {
          //      driveToggle = true;
           // }

            //else if (backPressed && driveToggle == true) {
             //   driveToggle = false;
            //}

            //if (driveToggle == false) {
             //   leftGo = Range.clip(drive - turn, -.75, .75);
            //    rightGo = Range.clip(drive + turn, -.75, .75);
            //}

            //else {
                leftGo = Range.clip(drive + turn, -1, 1);
                rightGo = Range.clip(drive - turn, -1, 1);
            //    driveLastPass = backPressed;
            //}

            Julieo.frontLeft.setPower(-leftGo);
            Julieo.frontRight.setPower(-rightGo);
            Julieo.backLeft.setPower(-leftGo);
            Julieo.backRight.setPower(-rightGo);
            Julieo.liftRight.setPower(liftPower);
            Julieo.liftLeft.setPower(liftPower);
            //   if (gamepad2.left_trigger >= .4) {
            //      Julieo.intake.setPower(.4);
            // }
            //else if (gamepad2.right_trigger >= .4) {
            //   Julieo.intake.setPower(-.4);
            //}
            // else {
            //    Julieo.intake.setPower(0);
            //}
            // if (gamepad2.a) {
            //    Julieo.lift.setPower(.5);
            // }
            //  else if (gamepad2.b) {
            //     Julieo.lift.setPower(-.5);
            //}
            // else {
            //    Julieo.lift.setPower(0);
            // }
            // if (gamepad2.left_bumper) {
            //    Julieo.intake.setPower(1);
            //}
            // else if (gamepad2.right_bumper) {
            //    Julieo.intake.setPower(-1);
            //}
            // else {
            //    Julieo.intake.setPower(.01);
            ///}
            //if (gamepad2.y) {
            //   Julieo.reelIn.setPower(.4);
            // }
            //else if (gamepad2.x) {
            //   Julieo.reelIn.setPower(-.4);
            // }
            //else {
            //  Julieo.reelIn.setPower(0);
            // }


            if (gamepad2.a) {
                Julieo.bucket.setPosition(1);
            }
            else if (gamepad2.b) {
                Julieo.bucket.setPosition(0);
            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

