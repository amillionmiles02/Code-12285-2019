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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareEvent
{
    /* Public OpMode members. */
    public DcMotor  frontLeft;
    public DcMotor  frontRight;
    public DcMotor  backLeft;
    public DcMotor  backRight;
 //   public DcMotor  flipper;
  //  public CRServo intake;
//    public DcMotor reel;
 //   public Servo bucket;
    public static final double TURN_SPEED = .75;
    public static final double HALF_SPEED = .5;
    public static final double QUARTER_SPEED = .25;
    public static final double FULL_SPEED = 1;
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareEvent(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeft  = hwMap.get(DcMotor.class, "front_Left");
        frontRight = hwMap.get(DcMotor.class, "front_Right");
        backLeft = hwMap.get(DcMotor.class, "back_Left");
        backRight = hwMap.get(DcMotor.class, "back_Right");
    //    flipper = hwMap.get(DcMotor.class, "flipper");
     //   intake = hwMap.get(CRServo.class, "intake");
      //  reel = hwMap.get(DcMotor.class, "reel");
    //    bucket = hwMap.get(Servo.class, "bucket");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        
        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    flipper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //    reel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
}
/*
//public void intake() {
 //   intake.setPower(1);
//}
public void outtake() {
    intake.setPower(-1);
}
public void reelIn() {
    reel.setPower(.4);
}
public void flipOut() {
    flipper.setPower(.2);
}
public void flipIn() {
    flipper.setPower(-.2);
}
public void outFlip() { // the upper bucket
    bucket.setPosition(1);
}
public void inFlip() {
    bucket.setPosition(0);
}
*/
public void turnLeft() {
        frontLeft.setPower(-TURN_SPEED);
        frontRight.setPower(TURN_SPEED);
        backLeft.setPower(-TURN_SPEED);
        backRight.setPower(TURN_SPEED);
    }
public void turnRight() {
        frontLeft.setPower(TURN_SPEED);
        frontRight.setPower(-TURN_SPEED);
        backLeft.setPower(TURN_SPEED);
        backRight.setPower(-TURN_SPEED);
    }
public void driveStraight(){
        frontLeft.setPower(TURN_SPEED);
        frontRight.setPower(TURN_SPEED);
        backLeft.setPower(TURN_SPEED);
        backRight.setPower(TURN_SPEED);
    }
    public void stopMotors(){
        frontLeft.setPower(0); 
        frontRight.setPower(0); 
        backLeft.setPower(0); 
        backRight.setPower(0); 
    
    }
   
}


