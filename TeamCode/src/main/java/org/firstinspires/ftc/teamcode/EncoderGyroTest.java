/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="EncoderGyroTest", group="Pushbot")
@Disabled
public class EncoderGyroTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCompOne          Julieo  = new HardwareCompOne();
    Orientation angles;
    Acceleration gravity;    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
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

        // Send telemetry message to indicate successful Encoder reset


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  48,  48, 4.0);  // S1: Forward 47 Inches with 5 Sec timeout
        turn(90);
        encoderDrive(DRIVE_SPEED, -12, -12, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout


        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        telemetry.update();
        int newLeftTarget;
        int newRightTarget;
        int newBackRight, newBackLeft;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = Julieo.frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = Julieo.frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackLeft = Julieo.backLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRight = Julieo.backRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            Julieo.frontLeft.setTargetPosition(newLeftTarget);
            Julieo.frontRight.setTargetPosition(newRightTarget);
            Julieo.backLeft.setTargetPosition(newBackLeft);
            Julieo.backRight.setTargetPosition(newBackRight);

            // Turn On RUN_TO_POSITION
            Julieo.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Julieo.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Julieo.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Julieo.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();

            double target;
            double leftSpeed = 0;
            double rightSpeed = 0;

            double currentPos = Julieo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            target = 0;

            leftSpeed = (0.5 + (currentPos - target) / 100);
             rightSpeed =  (0.5 - (currentPos - target) / 100);
            Julieo.frontRight.setPower(rightSpeed);
            Julieo.frontLeft.setPower(leftSpeed);
            Julieo. backRight.setPower(rightSpeed);
            Julieo.backLeft.setPower(leftSpeed);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (Julieo.frontLeft.isBusy() && Julieo.frontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            Julieo.frontLeft.getCurrentPosition(),
                                            Julieo.frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            Julieo.frontLeft.setPower(0);
            Julieo.frontRight.setPower(0);
            Julieo.backLeft.setPower(0);
            Julieo.backRight.setPower(0);
            // Turn off RUN_TO_POSITION
            Julieo.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Julieo.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Julieo.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Julieo.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            //  sleep(250);   // optional pause after each move
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
    public void driveStraight(double speed, int direction) {
    double target;
        double leftSpeed = 0;
        double rightSpeed = 0;

        double currentPos = angles.firstAngle;
        target = 0;

        if (direction == 1) {
            leftSpeed = 0.5 - (currentPos - target) / 100;
            rightSpeed = (0.5 + (currentPos - target) / 100); // inverse
        }

        else if (direction == -1) {
            leftSpeed = (0.5 + (currentPos - target) / 100);
            rightSpeed = (0.5 - (currentPos - target) / 100);
        }

        /*
         * // Left - negative - current pos = -1, target = 0 leftSpeed = 0.5; rightSpeed
         * = -0.5; /* LF.set((.5 + rotateToAngleRate) * 0.5); LB.set((.5 +
         * rotateToAngleRate) * 0.5); RF.set((-.5 - rotateToAngleRate) * 0.5);
         * RB.set((-.5 - rotateToAngleRate) * 0.5);
         */

        if (direction == 1) {
            Julieo.frontLeft.setPower(leftSpeed);
            Julieo.backLeft.setPower(leftSpeed);
             Julieo.frontRight.setPower(-rightSpeed);
            Julieo.backRight.setPower(-rightSpeed);
        }

        else if (direction == -1) {
            Julieo.frontLeft.setPower(-leftSpeed);
            Julieo.backLeft.setPower(-leftSpeed);
            Julieo.frontRight.setPower(rightSpeed);
            Julieo.backRight.setPower(rightSpeed);
        }
}}
