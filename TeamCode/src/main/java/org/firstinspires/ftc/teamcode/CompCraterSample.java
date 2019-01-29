package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "CompCraterStraight", group = "Concept")
public class CompCraterSample extends LinearOpMode {
    HardwareCompOne         robot   = new HardwareCompOne();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AV/e8+b/////AAABmbRBUthEFkS5q5tONky3fuwNqWXNRBwBnC3q2vF8GwMimLO+yqr79e9oDY+Bmevx9Wj74r1sMYLMo+lhSvkJpF9WT9TviOGt4kWE/Zqx+0ERsfiIW9EPpYZklF+2CPIgAl91XiSjVGi6VSGY/hdCoPffP0XO4nT53RqGGAn3i3etEwL6Q9G4Yo3yW1xWOo+zSEJvHvKBHx12NdVHI1TvQnfANXFwawZZvQThLyHKwJpkKfsYar2QJrwsPhBTAdKhS0611lK/pchMfCRJgyAsJE452PALaLoFaO+L1TmKmGVEne/kH9Zk0Om+ND0sv5GKfH/fVENfayBRFsywpumngXz4mK8fZqSXOl9PEeYzmP5n";
    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            unlatch();
            drive();

            /*
            /** Activate Tensor Flow Object Detection.
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1) {
                                if (goldMineralX < silverMineral1X) {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    runCenter();
                                    sleep(1000000);
                                } else if (goldMineralX > silverMineral1X) {

                                    telemetry.addData("Gold Mineral Position", "Right");
                                    runRight();
                                    sleep(1000000);

                                }
                            }
                            else {
                                telemetry.addData("Gold Mineral Position", "Left");
                                runLeft();
                                sleep(1000000);

                            }
                        }
                        telemetry.update();
                    }
                } */

        }

        //  if (tfod != null) {
        //     tfod.shutdown();
        //}
        sleep(100000000);
    }
    public void drive() {
        robot.frontLeft.setPower(-1);
        robot.frontRight.setPower(-1);
        robot.backLeft.setPower(-1);
        robot.backRight.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path : Left", "Stopping", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path : Left", "Stopping", runtime.seconds());
            telemetry.update();
        }
        idle();
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public void unlatch() {
        // Step 1:  Getdown
        robot.liftLeft.setPower(-.6);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 7)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        // Step 2:  Spin right for 1.3 seconds
        robot.liftLeft.setPower(0);
        robot.frontLeft.setPower(0.23);
        robot.frontRight.setPower(-.23);
        robot.backLeft.setPower(.23);
        robot.backRight.setPower(-.23);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .5)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 3:  Drive Backwards for 1 Second
        robot.frontLeft.setPower(-0.23);
        robot.frontRight.setPower(-0.23);
        robot.backLeft.setPower(-0.23);
        robot.backRight.setPower(-0.23);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .5)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(-0.2);
        robot.frontRight.setPower(0.2);
        robot.backLeft.setPower(-0.2);
        robot.backRight.setPower(0.2);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .34)) {
            telemetry.addData("Path", "Leg 4: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        // Step 4:  Stop and close the claw.
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }
    public void runLeft() {
        robot.frontLeft.setPower(-0.3);
        robot.frontRight.setPower(0.3);
        robot.backLeft.setPower(-0.3);
        robot.backRight.setPower(0.3);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path : Left", "Leg 1:Turning Left", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path : Left", "Stopping", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0.3);
        robot.frontRight.setPower(0.3);
        robot.backLeft.setPower(0.3);
        robot.backRight.setPower(0.3);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path : Left", "Leg 2: Driving Forward", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path : Left", "Stopping", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0.3);
        robot.frontRight.setPower(-0.3);
        robot.backLeft.setPower(0.3);
        robot.backRight.setPower(-0.3);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path : Left", "Leg 3: Turning Right", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path : Left", "Stopping", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0.6);
        robot.frontRight.setPower(0.6);
        robot.backLeft.setPower(0.6);
        robot.backRight.setPower(0.6);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path : Left", "Stopping", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path : Left", "Stopping", runtime.seconds());
            telemetry.update();
        }

    }
    public void runCenter() {
        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(.5);
        robot.backLeft.setPower(.5);
        robot.backRight.setPower(.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path : Center", "Leg 1: Driving Straight", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path : Center", "Stopping", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(-0.5);
        robot.frontRight.setPower(.5);
        robot.backLeft.setPower(-.5);
        robot.backRight.setPower(.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path : Center", "Leg 2: Turning Left", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path : Center", "Stopping", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0.3);
        robot.frontRight.setPower(0.3);
        robot.backLeft.setPower(0.3);
        robot.backRight.setPower(0.3);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .5)) {
            telemetry.addData("Path : Center", "Leg 3: Go orward", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path : Center", "Stopping", runtime.seconds());
            telemetry.update();
        }
    }
    public void runRight() {
        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(-.5);
        robot.backLeft.setPower(.5);
        robot.backRight.setPower(-.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path : Right", "Leg 1: Turning Right", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(.5);
        robot.backLeft.setPower(.5);
        robot.backRight.setPower(.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path: Right", "Leg 2: Going Straight", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(-0.5);
        robot.frontRight.setPower(.5);
        robot.backLeft.setPower(-.5);
        robot.backRight.setPower(.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path: Right", "Leg 3: Turning Left", runtime.seconds());
            telemetry.update();
        }
        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(.5);
        robot.backLeft.setPower(.5);
        robot.backRight.setPower(.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path: Right", "Leg 4: Going Straight", runtime.seconds());
            telemetry.update();
        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path: Right", "Stopping", runtime.seconds());
            telemetry.update();
        }
        idle();
    }

}

