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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * ToDo: Vishana/Anshul - please document what this class does
 */

@Config
@TeleOp(name="Mrv_Manual", group="Manual mode")
// @Disabled
public class Mrv_Manual extends LinearOpMode {
    // Declare OpMode members.
    Mrv_Robot robot = new Mrv_Robot();

    static final double SERVO_POSITION = 0.5;
    static final double RANGE[] = {0.0, 1.0};
    static final int CYCLE_MS = 50;
    static final double INCREMENT = 0.25;

    private boolean rampUp = true;
    private double speedAdjust = 10;
    private ElapsedTime runtime = new ElapsedTime();
    //private double position = (RANGE[1] - RANGE[0]) / 2;
   // private double Wrist_pos = (RANGE[1] - RANGE[0]) / 2;
    //private double Finger_pos = RANGE[0];
    //double Arm_Power = 0;
    //double maxArm_Power = 0.5;
    //double shooter_power = 0.1;
    public static double duck_power = 0.5;

    int DuckPowerDir = 1;

    public static final String VUFORIA_LICENSE_KEY = "AZRnab7/////AAABmTUhzFGJLEyEnSXEYWthkjhGRzu8klNOmOa9WEHaryl9AZCo2bZwq/rtvx83YRIgV60/Jy/2aivoXaHNzwi7dEMGoaglSVmdmzPF/zOPyiz27dDJgLVvIROD8ww7lYzL8eweJ+5PqLAavvX3wgrahkOxxOCNeKG9Tl0LkbS5R11ATXL7LLWeUv5FP1aDNgMZvb8P/u96OdOvD6D40Nf01Xf+KnkF5EXwNQKk1r7qd/hiv9h80gvBXMFqMkVgUyogwEnlK2BfmeUhGVm/99BiwwW65LpKSaLVPpW/6xqz9SyPgZ/L/vshbWgSkTB/KoERiV8MsW79RPUuQS6NTOLY32I/kukmsis3MFst5LP/d3gx";


    //double intake_power = 0.9;
    //double Pusher_Pos = 0;
    //boolean shooterOn = false;
    //boolean intakeOn = false;
    boolean DuckOn = false;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {

        // Initialize the drive system vriables
        robot.init(hardwareMap);
        telemetry.addData("Status", "Init Hardware");
        telemetry.update();

     //   robot.setRunMode(Mrv_Robot.MrvMotors.ALL, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.setRunMode(Mrv_Robot.MrvMotors.ALL, DcMotor.RunMode.RUN_USING_ENCODER);
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(vuforia, 0);

        waitForStart();

        initMarvyn();

        telemetry.setAutoClear(false);

        while (opModeIsActive()) {
            mrvManualDrive();
            mrvDuckWheel();
        }
    }


    public void initMarvyn() {
        // set Arm positions
       // position = RANGE[1] - RANGE[0] / 2;
        // robot.Shooter_Servo.setPosition(position);

        //telemetry.addData("Pusher Position", Pusher_Pos);
        telemetry.addData("Status:", "MAAAAAAAAAARVYN initialized ;D");
        telemetry.update();

        return;
    }

    public void mrvManualDrive() {
        if (gamepad1.dpad_left && speedAdjust >= 1) {
            speedAdjust -= 1;
            telemetry.addData("Current speed: ", "%f", speedAdjust);
            telemetry.update();
        }

        if (gamepad1.dpad_right && speedAdjust <= 7) {
            speedAdjust += 1;
            telemetry.addData("Current speed: ", "%f", speedAdjust);
            telemetry.update();
        }

        //Lavanya 3 (the wonder algorithm!!!)
        robot.lower_left.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10)); // 1.0
        robot.lower_right.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10)); // 1.0
        robot.upper_left.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10)); // 0
        robot.upper_right.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10)); // 0

        return;
    }

    public void mrvDuckWheel() {
        //Shooter and pusher code
        if (gamepad2.left_bumper) {
            DuckOn = !DuckOn;
            telemetry.addData("Duck Wheel toggle to:", DuckOn);
            if (DuckOn) {
                telemetry.addData("Duck Wheel:", "Spinning Clockwise" );
                DuckPowerDir = 1;
            }
            sleep(500);
        }else if (gamepad2.right_bumper) {
            DuckOn = !DuckOn;
            telemetry.addData("Duck Wheel toggle to:", DuckOn);
            if (DuckOn) {
                telemetry.addData("Duck Wheel:", "Spinning Counterclockwise" );
                DuckPowerDir = -1;
            }
            sleep(500);
        }

        if (DuckOn) {
            robot.duck_wheel.setPower(duck_power * DuckPowerDir);
        } else {
            robot.duck_wheel.setPower(0);
        }

               telemetry.update();
        return;
    }
}

