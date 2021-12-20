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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;



@Config
@TeleOp(name="Mrv_Manual", group="Manual mode")
// @Disabled
public class Mrv_Manual extends LinearOpMode {
    // Declare OpMode members.
    Mrv_Robot marvyn = new Mrv_Robot();

    static final double INCREMENT   = 0.1;     // amount to slew servo each CYCLE_MS cycle
    static final double[] RANGE_FULL= {0.0, 1.0};

    // TODO: Remove these unused variables [Lavanya]
    static final int    CYCLE_MS    =   50;     // period of each cycle
    private boolean rampUp = true;
    Servo   activeServo = null;
    ServoController activeServoController = null;
    private ElapsedTime runtime = new ElapsedTime();
    boolean WristStraight=true;
    double WristPosition = 0.5;
    double[] range = RANGE_FULL;
    double increment = INCREMENT;

    // TODO: This used to be 5 before, why not 10? [Lavanya]
    private double speedAdjust = 10;
    private double actuatorAdjust = 5;
    private double winchAdjust = 5;

    // TODO: Fine tune the actual duck power to be able to deliver 9 ducks [Atiksh]
    public static double duck_power = 0.25;

    boolean ServoTurn = true;
    double  position = 0.5; // Start at halfway position
    public static double Wrist_Grab_Pos = 0.9;
    public static double Wrist_Down_Pos = 0.35;
    public static double Wrist_Start_Pos = 0.3;

    int DuckPowerDir = 1;
    public static final String VUFORIA_LICENSE_KEY = "AZRnab7/////AAABmTUhzFGJLEyEnSXEYWthkjhGRzu8klNOmOa9WEHaryl9AZCo2bZwq/rtvx83YRIgV60/Jy/2aivoXaHNzwi7dEMGoaglSVmdmzPF/zOPyiz27dDJgLVvIROD8ww7lYzL8eweJ+5PqLAavvX3wgrahkOxxOCNeKG9Tl0LkbS5R11ATXL7LLWeUv5FP1aDNgMZvb8P/u96OdOvD6D40Nf01Xf+KnkF5EXwNQKk1r7qd/hiv9h80gvBXMFqMkVgUyogwEnlK2BfmeUhGVm/99BiwwW65LpKSaLVPpW/6xqz9SyPgZ/L/vshbWgSkTB/KoERiV8MsW79RPUuQS6NTOLY32I/kukmsis3MFst5LP/d3gx";

    boolean DuckOn = false;
    FtcDashboard mrvDashboard;

    @Override
    public void runOpMode() {

        // Initialize the drive system vriables
        // TODO: Lavanya - move all this code (before waitforstart()) to initMarvyn [Lavanya]
        marvyn.init(hardwareMap);
        telemetry.addData("Status", "Init Hardware");
        telemetry.update();

        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        mrvDashboard = FtcDashboard.getInstance();
        mrvDashboard.startCameraStream(vuforia, 0);

        marvyn.The_Claw.setPosition(0);
        marvyn.Wristy.setPosition(Wrist_Start_Pos);

        waitForStart();

        // TODO: Why do we InitMarvyn after start? [Lavanya]
        initMarvyn();

        telemetry.setAutoClear(false);
        while (opModeIsActive()) {
            mrvManualDrive();
            mrvDuckWheel();
            mrvClaw();
            mrvLinAc();
            mrvDaWinch();
            mrvWrist();
        }
    }


    // TODO: What all do we need to initialize? Does the other TODO above apply to this? [Lavanya]
    public void initMarvyn() {

        telemetry.addData("Status:", "Marvyn initialized ;D");
        telemetry.update();

        return;
    }

    public void mrvManualDrive()
    {

        // TODO: Fix problem with Dpad left turning speed completely off [Satvika]
        //       Looks like when user presses
        //       D-Pad left, robot stops moving anymore. Pressing DPad right seems to make it move just fine again.
        if (gamepad1.dpad_left ) {
            speedAdjust -= 1;
            if(speedAdjust < 1){
                speedAdjust = 1;
            }
            telemetry.addData("Current speed: ", "%f", speedAdjust);
            telemetry.update();
        }

        if (gamepad1.dpad_right ) {
            speedAdjust += 1;
            if (speedAdjust > 10){
                speedAdjust = 10;
            }
            telemetry.addData("Current speed: ", "%f", speedAdjust);
            telemetry.update();
        }

        //Lavanya 3 (the wonder algorithm!!!)
       /* robot.lower_left.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10)); // 1.0
        robot.lower_right.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10)); // 1.0
        robot.upper_left.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10)); // 0
        robot.upper_right.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10)); // 0
        */

       float turnDir = gamepad2.left_stick_x + gamepad1.right_stick_x;
       float moveDir = gamepad1.left_stick_y;
       float strafeDir = gamepad1.left_stick_x;

       if (turnDir > 1){
           turnDir =1;
       }else if (turnDir < -1){
           turnDir = -1;
       }
        marvyn.lower_left.setPower((moveDir + strafeDir - turnDir) * (-speedAdjust / 10)); // 1.0
        marvyn.lower_right.setPower((moveDir - strafeDir +turnDir) * (-speedAdjust / 10)); // 1.0
        marvyn.upper_left.setPower((moveDir - strafeDir - turnDir) * (-speedAdjust / 10)); // 0
        marvyn.upper_right.setPower((moveDir + strafeDir +turnDir) * (-speedAdjust / 10)); // 0

        return;
    }

    public void mrvDuckWheel() {
        //Shooter and pusher code

        if (gamepad2.left_bumper) {
            DuckOn = gamepad2.left_bumper;
            telemetry.addData("Duck Wheel toggle to:", DuckOn);
            if (DuckOn) {
                telemetry.addData("Duck Wheel:", "Spinning Clockwise" );
                DuckPowerDir = 1;
            }
            sleep(500);
        }
        else if (gamepad2.right_bumper) {
            DuckOn = gamepad2.right_bumper;
            telemetry.addData("Duck Wheel toggle to:", DuckOn);
            if (DuckOn) {
                telemetry.addData("Duck Wheel:", "Spinning Counterclockwise" );
                DuckPowerDir = -1;
            }
            sleep(500);
        }
        else {
            DuckOn = false;
        }

        if (DuckOn) {
            marvyn.duck_wheel.setPower(duck_power * DuckPowerDir);
        }
        else {
            marvyn.duck_wheel.setPower(0);
        }

        telemetry.update();
        return;
    }

    public void mrvClaw() {
        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.

        ServoTurn = gamepad2.right_trigger == 1f;
        // slew the servo, according to the rampUp (direction) variable.
        if (ServoTurn) {
            // Keep stepping up until we hit the max value.
            //position += increment ;
            //if (position >= 1 ) {
            position = 1;
            //}
        } else {
            // Keep stepping down until we hit the min value.
            //position -= increment ;
            //if (position <= 0 ) {
            position = 0;
            //}
        }

        // Set the servo to the new position and pause;
        marvyn.The_Claw.setPosition(position);

        //telemetry.addData("RampUp", ServoTurn);
    }

    public void mrvWrist()
    {
        // TODO: Use state tracking to implement button control. One method tracks the state, one method ensures action [Lavanya]
        //       Trigger is not the right control to move the Wrist
        //       Trigger should be used like the joy sticks to keep something happening. Think hand drill
        //       As long as the trigger is pressed, the robot keeps doing something and the degree to which it is
        //       pressed controls the intensity. Trigger is a bit similar to joystick, but only provides a value
        //       between 0 - 1 rather than -1 - 1 like a joystick.

        if(gamepad2.left_trigger == 1f) {
            //WristStraight = !WristStraight;
            marvyn.Wristy.setPosition(Wrist_Down_Pos);
        }
        else
        {
            marvyn.Wristy.setPosition(Wrist_Grab_Pos);
        }

        return;

    }

    // TODO: Add a limit switch to Linear actuator - touch sensor? [Atiksh/Anshul]
        public void mrvLinAc()
        {
            if (gamepad2.dpad_left ) {
                actuatorAdjust -= 1;
                if (actuatorAdjust < 1){
                    actuatorAdjust =1;
                }
                telemetry.addData("Current speed: ", "%f", actuatorAdjust);
                telemetry.update();
            }

            if (gamepad2.dpad_right) {
                actuatorAdjust += 1;
                if (actuatorAdjust > 10){
                    actuatorAdjust = 10;
                }
                telemetry.addData("Current speed: ", "%f", actuatorAdjust);
                telemetry.update();
            }
            marvyn.Linac_2.setPower(gamepad2.left_stick_y * (actuatorAdjust/10));
        }

        // TODO: Add a limit switch to Da Winch - Touch sensor? [Satvika/Vishruth]
        public void mrvDaWinch()
        {
            if (gamepad2.dpad_up ) {
                winchAdjust -= 1;
                if (winchAdjust < 1){
                    winchAdjust = 1;
                }
                telemetry.addData("Current speed: ", "%f", winchAdjust);
                telemetry.update();
            }

            if (gamepad2.dpad_down && winchAdjust <= 10) {
                winchAdjust += 1;
                if (winchAdjust > 1){
                    winchAdjust = 10;
                }
                telemetry.addData("Current speed: ", "%f", winchAdjust);
                telemetry.update();
            }
            marvyn.Da_Winch.setPower(gamepad2.right_stick_y * (winchAdjust/10));
        }
    }

