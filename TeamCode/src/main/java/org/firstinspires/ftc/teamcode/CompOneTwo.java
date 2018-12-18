package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

        Julieo.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Julieo.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Julieo.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Julieo.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Julieo.liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Julieo.liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            double leftGo = -gamepad1.left_stick_y;
            double rightGo = -gamepad1.right_stick_y;
            double liftPower = -gamepad2.left_stick_y;
        //    double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            boolean backPressed = gamepad1.x;
            boolean driveLastPass = false;
            boolean driveToggle = false;


            liftPower = Range.clip(liftPower, -.6, .6);

         //   if (backPressed && driveToggle == false) {
          //      driveToggle = true;
           // }

            //else if (backPressed && driveToggle == true) {
             //   driveToggle = false;
            //}

            //if (driveToggle == false) {
              //  leftGo = Range.clip(drive - turn, -.75, .75);
             //   rightGo = Range.clip(drive + turn, -.75, .75);
            //}

            //else {
             //   leftGo = Range.clip(drive + turn, -1, 1);
              //  rightGo = Range.clip(drive - turn, -1, 1);
            //    driveLastPass = backPressed;
            //}
            Julieo.frontLeft.setPower(leftGo);
            Julieo.frontRight.setPower(-leftGo);
            Julieo.backLeft.setPower(-rightGo);
            Julieo.backRight.setPower(rightGo);
            Julieo.liftRight.setPower(liftPower);
            Julieo.liftLeft.setPower(liftPower);





            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

