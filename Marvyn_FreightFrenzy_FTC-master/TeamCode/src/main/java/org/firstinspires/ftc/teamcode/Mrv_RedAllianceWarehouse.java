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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Config
@Autonomous(name="Red Warehouse", group="Autonomous Mode")
@Disabled
public class Mrv_RedAllianceWarehouse extends LinearOpMode {

    Mrv_Robot robot = new Mrv_Robot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    double DRIVE_SPEED = 5;
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.779;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public static double left_inches = 12;
    public static double right_inches = 12;
    public static double drive_speed = 1;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            "AZRnab7/////AAABmTUhzFGJLEyEnSXEYWthkjhGRzu8klNOmOa9WEHaryl9AZCo2bZwq/rtvx83YRIgV60/Jy/2aivoXaHNzwi7dEMGoaglSVmdmzPF/zOPyiz27dDJgLVvIROD8ww7lYzL8eweJ+5PqLAavvX3wgrahkOxxOCNeKG9Tl0LkbS5R11ATXL7LLWeUv5FP1aDNgMZvb8P/u96OdOvD6D40Nf01Xf+KnkF5EXwNQKk1r7qd/hiv9h80gvBXMFqMkVgUyogwEnlK2BfmeUhGVm/99BiwwW65LpKSaLVPpW/6xqz9SyPgZ/L/vshbWgSkTB/KoERiV8MsW79RPUuQS6NTOLY32I/kukmsis3MFst5LP/d3gx";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static int duck_pos;

    @Override
    public void runOpMode()
    {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);
        telemetry.setAutoClear(false);
        telemetry.addData("Status", "Init Hardware");
        telemetry.update();

        robot.setRunMode(Mrv_Robot.MrvMotors.ALL, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(Mrv_Robot.MrvMotors.ALL, DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status: ", "Initializing camera......");
        telemetry.update();

        telemetry.addData("Status: ", "Ready");
        telemetry.update();


        // Init Tensorflow
        initVuforia();
        initTfod();

        if(tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5,16.0/9.0);
        }

        waitForStart();

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    i++;
                    if (recognition.getLabel() == "Duck") {
                        duck_pos = 1;
                        break;
                    }
                }
            }
        }
        // Raise the wrist
        // DuckPos = GetDuckPosition();
        // FreightDropOff(DuckPos);
        runtime.reset();
        telemetry.setAutoClear(true);
        Freight_dropoff(duck_pos);
        // For blue 1 , Turn left and go 30 inches
        // For blue 2, Turn left and go 90 inches
        // For red 1, Turn right and go 30 inches
        // For red 2 , Turn right and go 90 inches

        //blue 1
            //turn 90 degrees
       // mrvAutoDrive(drive_speed,left_inches,right_inches,1000);
            //strafe to wall
//        mrvAutoStrafe(0.5,-12,12,1000);
            //drive forward
        mrvAutoDrive(0.5, 26 ,26,5000);
        mrvAutoStrafe(0.5,25, 25, 5000);

    }


    public void Freight_dropoff( double duck_pos)
    {
        if(duck_pos == 1) {
            // move forward
            // raise arm
            // Extend actuator
            // drop freight
        }
        else if(duck_pos ==2 ) {
            // Turn rigt
            // Raise arm
            // Extend actuator
            // drop freight
        }
        else if(duck_pos == 3)
        {
            // Turn left
            // Raise arm
            // Extend actuator
            // drop freight
        }
        else
            return;

    }

    public void dropBlockOnTop() {
        if (opModeIsActive()) {

            robot.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setRunMode(Mrv_Robot.MrvMotors.LIN_AC, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, DcMotor.RunMode.RUN_USING_ENCODER);
            robot.setRunMode(Mrv_Robot.MrvMotors.LIN_AC, DcMotor.RunMode.RUN_USING_ENCODER);
//
            robot.setTargetPosition(Mrv_Robot.MrvMotors.LIN_AC, 513);
            robot.setPower(Mrv_Robot.MrvMotors.LIN_AC, 1);
            robot.setRunMode(Mrv_Robot.MrvMotors.LIN_AC, DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && robot.areMotorsBusy(Mrv_Robot.MrvMotors.LIN_AC)) {
                telemetry.addData("Status: ", "Arm moving forward");
                telemetry.update();
            }


           /* robot.setTargetPosition(Mrv_Robot.MrvMotors.WINCH, 700/2);
            robot.setPower(Mrv_Robot.MrvMotors.WINCH, 0.4);
            robot.setRunMode(Mrv_Robot.MrvMotors.WINCH, DcMotor.RunMode.RUN_TO_POSITION);
            while(opModeIsActive() && robot.areMotorsBusy(Mrv_Robot.MrvMotors.WINCH)) {
                telemetry.addData("Status: ","Arm moving forward");
                telemetry.update();
            }


            sleep(1000);
            robot.The_Claw.setPosition(1);
            sleep(1000);
            robot.The_Claw.setPosition(0);

            telemetry.addData("Arm Motor position1:", robot.getCurrentPosition(Mrv_Robot.MrvMotors.WINCH));
            robot.setTargetPosition(Mrv_Robot.MrvMotors.WINCH, 0);
            robot.setPower(Mrv_Robot.MrvMotors.WINCH, 1);
            while(opModeIsActive() && robot.areMotorsBusy(Mrv_Robot.MrvMotors.WINCH)) {
                telemetry.addData("Status: ","Arm moving backward");
                telemetry.update();
            }
            robot.setPower(Mrv_Robot.MrvMotors.WINCH,0);

            telemetry.addData("Arm Motor position2:", robot.getCurrentPosition(Mrv_Robot.MrvMotors.WINCH));
            telemetry.addData("Status", "Dropping Wobble goal on field");
            telemetry.update();
            mrvAutoDrive(DRIVE_SPEED,6,-6, 30);
        }
*/
            return;
        }
    }



/*
     *  Method to perform a relative move, based on encoder counts.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *  4) Any of the motors stops
     */

    public void mrvAutoDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS)
    {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Reset
            robot.setRunMode(Mrv_Robot.MrvMotors.ALL, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setRunMode(Mrv_Robot.MrvMotors.ALL, DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newRightTarget = (int)(leftInches  * COUNTS_PER_INCH);
            newLeftTarget  = (int)(rightInches * COUNTS_PER_INCH);

            robot.setTargetPosition(Mrv_Robot.MrvMotors.UPPER_LEFT, newLeftTarget);
            robot.setTargetPosition(Mrv_Robot.MrvMotors.LOWER_RIGHT, newRightTarget);
            robot.setTargetPosition(Mrv_Robot.MrvMotors.UPPER_RIGHT, newRightTarget);
            robot.setTargetPosition(Mrv_Robot.MrvMotors.LOWER_LEFT, newLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.setRunMode(Mrv_Robot.MrvMotors.ALL_DRIVES, DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.setPower(Mrv_Robot.MrvMotors.ALL_DRIVES, Math.abs(speed));

            while (opModeIsActive() &&
                  (runtime.seconds() < timeoutS) &&
                  (robot.areMotorsBusy(Mrv_Robot.MrvMotors.ALL_DRIVES)) )
            {
                telemetry.addData("Target positions", "Left %7d : Right %7d", newLeftTarget, newRightTarget);
                telemetry.addData("Current  position", "Upper_Left %7d : Lower_Right %7d : Upper_Right %7d : Lower_Left %7d",
                        robot.getCurrentPosition(Mrv_Robot.MrvMotors.UPPER_LEFT),
                        robot.getCurrentPosition(Mrv_Robot.MrvMotors.LOWER_RIGHT),
                        robot.getCurrentPosition(Mrv_Robot.MrvMotors.UPPER_RIGHT),
                        robot.getCurrentPosition(Mrv_Robot.MrvMotors.LOWER_LEFT));
                telemetry.update();
            }

            // Stop all motion;
            robot.setPower(Mrv_Robot.MrvMotors.ALL_DRIVES, 0);

            // Turn off RUN_TO_POSITION
            robot.setRunMode(Mrv_Robot.MrvMotors.ALL_DRIVES, DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    public void mrvAutoStrafe(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        robot.setRunMode(Mrv_Robot.MrvMotors.ALL, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(Mrv_Robot.MrvMotors.ALL, DcMotor.RunMode.RUN_USING_ENCODER);

        if (opModeIsActive()) {
            newLeftTarget = robot.getCurrentPosition(Mrv_Robot.MrvMotors.LOWER_LEFT) + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.getCurrentPosition(Mrv_Robot.MrvMotors.LOWER_RIGHT) + (int) (rightInches * COUNTS_PER_INCH);

                 //    Strafe Right ->      Strafe Left <-
                 //    ^^  O-----O vv       vv  O-----O ^^
                 //           |                    |
                 //           |                    |
                 //    vv  O-----O ^^       ^^  O-----O vv

            robot.setTargetPosition(Mrv_Robot.MrvMotors.LOWER_LEFT, newLeftTarget);
            robot.setTargetPosition(Mrv_Robot.MrvMotors.UPPER_RIGHT, newRightTarget);

            robot.setTargetPosition(Mrv_Robot.MrvMotors.LOWER_RIGHT, -newRightTarget);
            robot.setTargetPosition(Mrv_Robot.MrvMotors.UPPER_LEFT, -newLeftTarget);


            runtime.reset();            robot.setRunMode(Mrv_Robot.MrvMotors.ALL_DRIVES, DcMotor.RunMode.RUN_TO_POSITION);

            robot.setPower(Mrv_Robot.MrvMotors.ALL_DRIVES, Math.abs(speed));

            while (opModeIsActive()
                    && runtime.seconds() < timeoutS
                    && robot.areMotorsBusy(Mrv_Robot.MrvMotors.ALL_DRIVES)) {
                telemetry.addData("Target positions", "Left %7d : Right %7d", newLeftTarget, newRightTarget);
                telemetry.addData("Current position", "Upper_Left %7d : Lower_Right %7d : Upper_Right %7d : Lower_Left %7d",
                        robot.getCurrentPosition(Mrv_Robot.MrvMotors.UPPER_LEFT),
                        robot.getCurrentPosition(Mrv_Robot.MrvMotors.LOWER_RIGHT),
                        robot.getCurrentPosition(Mrv_Robot.MrvMotors.UPPER_RIGHT),
                        robot.getCurrentPosition(Mrv_Robot.MrvMotors.LOWER_LEFT));
                telemetry.update();
            }

            // Stop all motion
            robot.setPower(Mrv_Robot.MrvMotors.ALL_DRIVES, 0);
            robot.setRunMode(Mrv_Robot.MrvMotors.ALL_DRIVES, DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }





}


