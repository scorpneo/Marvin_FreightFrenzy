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
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;


@Config
@TeleOp(name="Mrv_Manual", group="Manual mode")
//@Disabled
public class Mrv_Manual extends LinearOpMode {
    // Declare OpMode members.
    Mrv_Robot marvyn = new Mrv_Robot();

    static final double INCREMENT = 0.1;     // amount to slew servo each CYCLE_MS cycle
    static final double[] RANGE_FULL = {0.0, 1.0};

    public static double speedAdjust = 5;
    public static double linacAdjust = 10;
    public static double dawinchAdjust = 6;
    private boolean assumingPickPosition = false;
    private boolean assumingDropPosition = false;
    private boolean changingWheelSpeed = false;
    private boolean changingLinacSpeed = false;
    private boolean changingDaWinchiSpeed = false;
    private boolean changingWrist = false;

    public static double duck_power = 0.25;

    boolean ServoTurn = true;
    double position = 0.5; // Start at halfway position
    public static double Wrist_Parallel_to_Linac = 0.425; // Parallel to arm
    public static double Wrist_chute_dropoff = 0.85; // Perpendicular to Arm at top
    public static double Wrist_Pos = 0.3;
    public static double Wrist_Start_Pos = 0.3; // Perpendicular to Arm at bottom
    public static double Wrist_Pickup_Pos = 0.3;
    public static double Wrist_Dropoff_Pos = 0.4;
    public static double wrist_increment = 0.05;


    //0.52 for picking up preset
    public static double Claw_Open_Pos = 0.4;
    public static double Claw_Close_Pos = 0.0;
    public static int Winch_Parallel_to_ground = 100;
    public static int Linac_Parallel_to_ground = 100;
    public static int Winch_Chute_Dropoff = 100;
    public static int Linac_Chute_Dropoff = 100;

    public static int BUTTON_TRIGGER_TIMER_MS = 500;
    public static int Wristy_Button_Trigger_Timer_Ms = 85;
    public static int Linac_Grab_Position_Ticks = 288; // From REV Robotics Core HEX
    public static int Dawinchi_Grab_Position_Ticks = 1120; // From REV Robotics HD HEX 40:1

    private static ElapsedTime timer_gp1_buttonA;
    private static ElapsedTime timer_gp1_buttonX;
    private static ElapsedTime timer_gp1_buttonY;
    private static ElapsedTime timer_gp1_buttonB;
    private static ElapsedTime timer_gp1_dpad_up;
    private static ElapsedTime timer_gp1_dpad_down;
    private static ElapsedTime timer_gp1_dpad_left = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp1_dpad_right = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonA = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonX;
    private static ElapsedTime timer_gp2_buttonY;
    private static ElapsedTime timer_gp2_buttonB = new ElapsedTime(MILLISECONDS);
    ;
    private static ElapsedTime timer_gp2_dpad_up = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_down = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_left = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_right = new ElapsedTime(MILLISECONDS);


    int DuckPowerDir = 1;
    public static final String VUFORIA_LICENSE_KEY = "AZRnab7/////AAABmTUhzFGJLEyEnSXEYWthkjhGRzu8klNOmOa9WEHaryl9AZCo2bZwq/rtvx83YRIgV60/Jy/2aivoXaHNzwi7dEMGoaglSVmdmzPF/zOPyiz27dDJgLVvIROD8ww7lYzL8eweJ+5PqLAavvX3wgrahkOxxOCNeKG9Tl0LkbS5R11ATXL7LLWeUv5FP1aDNgMZvb8P/u96OdOvD6D40Nf01Xf+KnkF5EXwNQKk1r7qd/hiv9h80gvBXMFqMkVgUyogwEnlK2BfmeUhGVm/99BiwwW65LpKSaLVPpW/6xqz9SyPgZ/L/vshbWgSkTB/KoERiV8MsW79RPUuQS6NTOLY32I/kukmsis3MFst5LP/d3gx";

    boolean DuckOn = false;
    FtcDashboard mrvDashboard;

    @Override
    public void runOpMode() {

        // Initialize the drive system vriables
        marvyn.init(hardwareMap);

        waitForStart();

        initMarvyn();

        while (opModeIsActive()) {
            mrvManualDrive();
            mrvDuckWheel();
            //mrvClaw();
            mrvLinAc();
            mrvDaWinchi();
            mrvWrist();
            mrvSwitchtoGrabPosition();
            mrvIntakeClaw();
            //mrvAppendagePresets();
        }
    }

    public void mrvIntakeClaw() {
        boolean bIntake = gamepad2.right_trigger == 1f;
        boolean bOuttake = gamepad2.left_trigger == 1f;

//        marvyn.killPowerIfStalled(Mrv_Robot.MrvServos.LEFT_CLAW);
//        marvyn.killPowerIfStalled(Mrv_Robot.MrvServos.RIGHT_CLAW);

        if (bIntake) {
            marvyn.Claw_Left.setPower(0.5);
            marvyn.Claw_Right.setPower(0.5);
        } else if (bOuttake) {
            marvyn.Claw_Left.setPower(-0.5);
            marvyn.Claw_Right.setPower(-0.5);
        } else {
            marvyn.Claw_Left.setPower(0);
            marvyn.Claw_Right.setPower(0);
        }

    }

    public void initMarvyn() {
        msStuckDetectStop = 2500;
        mrvDashboard = FtcDashboard.getInstance();

        // TODO: Make this work if drivers agree. (Calling this method does nothing because it checks for gamepad2.a)
        mrvSwitchtoGrabPosition();
        Wrist_Pos = Wrist_Start_Pos;

        telemetry.addData("Status:", "Marvyn ready to grab freight!");
        telemetry.update();

        return;
    }

    public void mrvManualDrive() {
        if (gamepad1.dpad_left) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_left.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (speedAdjust <= 1) {
                    speedAdjust = 1;
                } else {
                    speedAdjust -= 1;
                }
                telemetry.addData("Current speed: ", "%f", speedAdjust);
                telemetry.update();
                changingWheelSpeed = false;
            }
        }

        //gamepad right -> increase wheel speed
        if (gamepad1.dpad_right) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_right.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (speedAdjust >= 10) {
                    speedAdjust = 10;
                } else {
                    speedAdjust += 1;
                }
                telemetry.addData("Current speed: ", "%f", speedAdjust);
                telemetry.update();
                changingWheelSpeed = false;
            }
        }

        float turnDir = gamepad1.right_stick_x;
        float moveDir = gamepad1.left_stick_y;
        float strafeDir = gamepad1.left_stick_x;

        if (turnDir > 1) {
            turnDir = 1;
        } else if (turnDir < -1) {
            turnDir = -1;
        }
        marvyn.lower_left.setPower((moveDir + strafeDir - turnDir) * (-speedAdjust / 10)); // 1.0
        marvyn.lower_right.setPower((moveDir - strafeDir + turnDir) * (-speedAdjust / 10)); // 1.0
        marvyn.upper_left.setPower((moveDir - strafeDir - turnDir) * (-speedAdjust / 10)); // 0
        marvyn.upper_right.setPower((moveDir + strafeDir + turnDir) * (-speedAdjust / 10)); // 0

        return;
    }

    public void mrvDuckWheel() {

        if (gamepad2.left_bumper) {
            DuckOn = gamepad2.left_bumper;
            telemetry.addData("Duck Wheel toggle to:", DuckOn);
            if (DuckOn) {
                telemetry.addData("Duck Wheelsss:", "Spinning Clockwise");
                telemetry.update();
                DuckPowerDir = 1;
            }
            sleep(500);
        } else if (gamepad2.right_bumper) {
            DuckOn = gamepad2.right_bumper;
            telemetry.addData("Duck Wheel toggle to:", DuckOn);
            if (DuckOn) {
                telemetry.addData("Duck Wheelsss:", "Spinning Counterclockwise");
                telemetry.update();
                DuckPowerDir = -1;
            }
            sleep(500);
        } else {
            DuckOn = false;
        }

        if (DuckOn) {
            marvyn.setPower(Mrv_Robot.MrvMotors.DUCK_WHEELS, duck_power * DuckPowerDir);
        } else {
            marvyn.setPower(Mrv_Robot.MrvMotors.DUCK_WHEELS, 0);
        }
        return;
    }

    public void mrvClaw() {
        ServoTurn = gamepad2.right_trigger == 1f;
        // slew the servo, according to the rampUp (direction) variable.
        if (ServoTurn) {
            position = Claw_Open_Pos;
        } else {
            position = Claw_Close_Pos;
        }

    }

    public void mrvWrist() {
        // if (gamepad2.left_trigger == 1f) {
        //  Wrist_Pos = Wrist_chute_dropoff;
        //} else
        if (gamepad2.dpad_down) {
            if (!changingWrist) {
                timer_gp2_dpad_down.reset();
                changingWrist = true;
            } else if (timer_gp2_dpad_down.time(TimeUnit.MILLISECONDS) > Wristy_Button_Trigger_Timer_Ms) {
                if (Wrist_Pos <= Wrist_Start_Pos) {
                    Wrist_Pos = Wrist_Start_Pos;
                } else {
                    Wrist_Pos -= wrist_increment;
                }
                telemetry.addData("Wrist Pos: ", "%f", Wrist_Pos);
                telemetry.update();
                changingWrist = false;
            }
        } else if (gamepad2.dpad_up) {
            if (!changingWrist) {
                timer_gp2_dpad_up.reset();
                changingWrist = true;
            } else if (timer_gp2_dpad_up.time(TimeUnit.MILLISECONDS) > Wristy_Button_Trigger_Timer_Ms) {
                if (Wrist_Pos >= Wrist_chute_dropoff) {
                    Wrist_Pos = Wrist_chute_dropoff;
                } else {
                    Wrist_Pos += wrist_increment;
                }
                telemetry.addData("Wrist Pos: ", "%f", Wrist_Pos);
                telemetry.update();
                changingWrist = false;
            }
        }

        if (gamepad2.a) {
            if (!assumingPickPosition) {
                timer_gp2_buttonA.reset();
                assumingPickPosition = true;
            } else if (timer_gp2_buttonA.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_A triggered. Will set Wrist to Pickup position");
                telemetry.update();
                Wrist_Pos = Wrist_Pickup_Pos;
                //   marvyn.The_Claw.setPosition(0);
                assumingPickPosition = false;
            }
        }

        if (gamepad2.b) {
            if (!assumingDropPosition) {
                timer_gp2_buttonB.reset();
                assumingDropPosition = true;
            } else if (timer_gp2_buttonA.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_B triggered. Will set Wrist to Pickup position");
                telemetry.update();
                Wrist_Pos = Wrist_Dropoff_Pos;
                assumingDropPosition = false;
            }
        } else {
            assumingDropPosition = false;
        }
        //default
        //  else{
        //     Wrist_Pos = Wrist_Parallel_to_Linac;
        // }

        marvyn.Wristy.setPosition(Wrist_Pos);
        return;
    }

    public void mrvLinAc() {
        if (!marvyn.Touche.getState() == true) {
            if (-gamepad2.left_stick_y < 0) {
                marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 0);
            } else {
                marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, -gamepad2.left_stick_y * (linacAdjust / 10));
            }
        } else {
            marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, -gamepad2.left_stick_y * (linacAdjust / 10));
        }
    }

    public void mrvDaWinchi() {
        marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, gamepad2.right_stick_y * (dawinchAdjust / 10));
    }


    public void mrvSwitchtoGrabPosition() {
    }
}









