package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


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
    Orientation angles;
    Acceleration gravity;
    ElapsedTime                 runtime = new ElapsedTime();
    // three times the necessary expense.

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
        //Julieo.liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        // three times the necessary expense.
        composeTelemetry();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            double leftGo = -gamepad1.left_stick_y;
            double rightGo = -gamepad1.right_stick_y;
            double liftPower = -gamepad2.left_stick_y;
        //    double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            boolean backPressed = gamepad1.a;
            boolean driveLastPass = false;
            boolean driveToggle = false;
            boolean speedToggle = false;

            liftPower = Range.clip(liftPower, -.6, .6);

            if (gamepad1.left_trigger > .2 && driveToggle == false) {
                driveToggle = true;
            }

            else if (gamepad1.left_trigger > .2 && driveToggle == true) {
                driveToggle = false; }

            if (driveToggle == false) {
                Julieo.frontLeft.setPower(-rightGo);
                Julieo.frontRight.setPower(-leftGo);
                Julieo.backLeft.setPower(-rightGo);
                Julieo.backRight.setPower(-leftGo);
                telemetry.addLine()
                        .addData("Alignment:", " Reversed");
            }

            else {
                Julieo.frontLeft.setPower(rightGo);
                Julieo.frontRight.setPower(leftGo);
                Julieo.backLeft.setPower(rightGo);
                Julieo.backRight.setPower(leftGo);
                telemetry.addLine()
                        .addData("Alignment:", " Normal");
                driveLastPass = backPressed;
            }
            if (gamepad1.right_trigger > .2 && speedToggle == false) {
                speedToggle = true;
            }

            else if (gamepad1.right_trigger > .2 && speedToggle == true) {
                speedToggle = false; }

            if (speedToggle == false) {
                Julieo.frontLeft.setPower(Julieo.frontLeft.getPower());
                Julieo.frontRight.setPower(Julieo.frontLeft.getPower());
                Julieo.backLeft.setPower(Julieo.backLeft.getPower());
                Julieo.backRight.setPower(Julieo.backRight.getPower());
                telemetry.addLine()
                        .addData("Speed:", " Normal");

            }

            else {
                Julieo.frontLeft.setPower(Julieo.frontLeft.getPower() * .5);
                Julieo.frontRight.setPower(Julieo.frontLeft.getPower() * .5);
                Julieo.backLeft.setPower(Julieo.backLeft.getPower() * .5);
                Julieo.backRight.setPower(Julieo.backRight.getPower() * .5);
                telemetry.addLine()
                        .addData("Speed:", " Reduced");
                driveLastPass = backPressed;
            }

            //Julieo.liftRight.setPower(liftPower);
            Julieo.liftLeft.setPower(liftPower);
            if (gamepad2.dpad_down) {
                Julieo.knocker.setPosition(1);
            }
            else if (gamepad2.dpad_up) {
                Julieo.knocker.setPosition(Julieo.START);
            }
telemetry.addLine()
            .addData("Left Motor", leftGo)
            .addData("Right Motor", rightGo)
            .addData("Servo", Julieo.knocker.getPosition());
        telemetry.update();


        }

    }



    public void turn(double target) {
        if(angles.firstAngle > target){
            Julieo.frontLeft.setPower( .5);
            Julieo.frontRight.setPower(.5);
            Julieo.backLeft.setPower(-.5);
            Julieo.backRight.setPower(-.5);
        }
        else {
            Julieo.frontLeft.setPower(0);
            Julieo.frontRight.setPower(0);
            Julieo.backLeft.setPower(0);
            Julieo.backRight.setPower(0);
        }
        angles.firstAngle = 0;
        telemetry.update();
    }
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = Julieo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = Julieo.imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return Julieo.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return Julieo.imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    }
    /*
    private void resetAngle()
    {
        Julieo.angles = Julieo.imu.getAngularOrientation();

        globalAngle = 0;
    }*/

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    /*private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation current_angles = Julieo.imu.getAngularOrientation();

        double deltaAngle = current_angles.firstAngle - Julieo.angles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        Julieo.angles = current_angles;

        return globalAngle;
    }*/
    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    /*private void turn(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        Julieo.frontLeft.setPower(leftPower);
        Julieo.backRight.setPower(rightPower);
        Julieo.backLeft.setPower(leftPower);
        Julieo.frontRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        Julieo.frontLeft.setPower(0);
        Julieo.backRight.setPower(0);
        Julieo.backLeft.setPower(0);
        Julieo.frontRight.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.

    } */



