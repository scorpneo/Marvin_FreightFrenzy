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

    // TODO: Remove these unused variables [Lavanya]
    //   static final int    CYCLE_MS    =   50;     // period of each cycle
    //  private boolean rampUp = true;
    // Servo   activeServo = null;
    //ServoController activeServoController = null;
    //private ElapsedTime runtime = new ElapsedTime();
    //boolean WristStraight=true;
    //double WristPosition = 0.5;
    //double[] range = RANGE_FULL;
    //double increment = INCREMENT;

    // TODO: This used to be 5 before, why not 10? [Lavanya]
    public static double speedAdjust = 5;
    public static double linacAdjust = 10;
    public static double dawinchAdjust = 8;
    private boolean assumingGrabPosition = false;
    private boolean changingWheelSpeed = false;
    private boolean changingLinacSpeed = false;
    private boolean changingDaWinchiSpeed = false;

    // TODO: Fine tune the actual duck power to be able to deliver 9 ducks [Atiksh]
    public static double duck_power = 0.25;

    boolean ServoTurn = true;
    double position = 0.5; // Start at halfway position
    public static double Wrist_Parallel_to_Linac = 0.425; // Parallel to arm
    public static double Wrist_chute_dropoff = 0.85; // Perpendicular to Arm at top
    public static double Wrist_Start_Pos = 0.0; // Perpendicular to Arm at bottom

    //0.52 for picking up preset
    public static double Claw_Open_Pos = 0.4;
    public static double Claw_Close_Pos = 0.0;
    public static int Winch_Parallel_to_ground = 100;
    public static int Linac_Parallel_to_ground = 100;
    public static int Winch_Chute_Dropoff = 100;
    public static int Linac_Chute_Dropoff = 100;

    public static int BUTTON_TRIGGER_TIMER_MS = 500;
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
    private static ElapsedTime timer_gp2_buttonB;
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
        // TODO: Lavanya - move all this code (before waitforstart()) to initMarvyn [Lavanya]
        marvyn.init(hardwareMap);

        waitForStart();
        initMarvyn();

        // TODO: Why do we InitMarvyn after start? [Lavanya]
        while (opModeIsActive()) {
            mrvManualDrive();
            mrvDuckWheel();
            mrvClaw();
            mrvLinAc();
            mrvDaWinchi();
            mrvWrist();
            mrvSwitchtoGrabPosition();
            //mrvAppendagePresets();
        }
    }


    // TODO: What all do we need to initialize? Does the other TODO above apply to this? [Lavanya]
    public void initMarvyn() {
        msStuckDetectStop = 2500;
        mrvDashboard = FtcDashboard.getInstance();

        marvyn.The_Claw.setPosition(0);
        marvyn.Wristy.setPosition(Wrist_Start_Pos);

        // TODO: Make this work if drivers agree. (Calling this method does nothing because it checks for gamepad2.a)
        mrvSwitchtoGrabPosition();

        telemetry.addData("Status:", "Marvyn ready to grab freight!");
        telemetry.update();

        return;
    }

    public void mrvManualDrive() {
        // TODO: Fix problem with Dpad left turning speed completely off [Satvika]
        //       Looks like when user presses
        //       D-Pad left, robot stops moving anymore. Pressing DPad right seems to make it move just fine again.

        //gamepad left -> decrease wheel speed
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

        public void mrvDuckWheel () {

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
                marvyn.setPower(Mrv_Robot.MrvMotors.DUCK_WHEELS,duck_power * DuckPowerDir);
            } else {
                marvyn.setPower(Mrv_Robot.MrvMotors.DUCK_WHEELS,0);
            }
            return;
        }




        public void mrvClaw ()
        {
            ServoTurn = gamepad2.right_trigger == 1f;
            // slew the servo, according to the rampUp (direction) variable.
            if (ServoTurn) {
                position = Claw_Open_Pos;
            } else {
                position = Claw_Close_Pos;
            }

            // Set the servo to the new position and pause;
            marvyn.The_Claw.setPosition(position);
        }

        public void mrvWrist ()
        {
            if (gamepad2.left_trigger == 1f) {
                marvyn.Wristy.setPosition(Wrist_chute_dropoff);
            } else {
                marvyn.Wristy.setPosition(Wrist_Parallel_to_Linac);
            }

            return;
        }

        // TODO: Add a limit switch to Linear actuator - touch sensor? [Atiksh/Anshul]
        public void mrvLinAc ()
        {

            //gamepad 2 dpad left -> decrease
            if (gamepad2.dpad_left) {
                if (!changingLinacSpeed) {
                    timer_gp2_dpad_left.reset();
                    changingLinacSpeed = true;
                } else if (timer_gp2_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                    if (linacAdjust <= 1) {
                        linacAdjust = 1;
                    } else {
                        linacAdjust -= 1;
                    }
                    telemetry.addData("linac power: ", "%f", linacAdjust);
                    telemetry.update();
                    changingLinacSpeed = false;
                }
            }


            //gamepad2 dpad right -> increase speed
            if (gamepad2.dpad_right) {
                if (!changingLinacSpeed) {
                    timer_gp2_dpad_right.reset();
                    changingLinacSpeed = true;
                } else if (timer_gp2_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                    if (linacAdjust >= 10) {
                        linacAdjust = 10;
                    } else {
                        linacAdjust += 1;
                    }
                    telemetry.addData("linac power: ", "%f", linacAdjust);
                    telemetry.update();
                    changingLinacSpeed = false;
                }
            }
            marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, gamepad2.left_stick_y * (linacAdjust / 10));
        }

        // TODO: Add a limit switch to Da Winch - Touch sensor? [Satvika/Vishruth]
        public void mrvDaWinchi ()
        {

           //gamepad2 dpad down -> decrease

            if (gamepad2.dpad_down) {
                if (!changingDaWinchiSpeed) {
                    timer_gp2_dpad_down.reset();
                    changingDaWinchiSpeed = true;
                } else if (timer_gp2_dpad_down.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                    if (dawinchAdjust <= 1) {
                        dawinchAdjust = 1;
                    } else {
                        dawinchAdjust -= 1;
                    }
                    telemetry.addData("dawinch power: ", "%f", dawinchAdjust);
                    telemetry.update();
                    changingDaWinchiSpeed = false;
                }
            }

            //gamepad2 dpad up
            if (gamepad2.dpad_up) {
                if (!changingDaWinchiSpeed) {
                    timer_gp2_dpad_up.reset();
                    changingDaWinchiSpeed = true;
                } else if (timer_gp2_dpad_up.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                    if (dawinchAdjust >= 10) {
                        dawinchAdjust = 10;
                    } else {
                        dawinchAdjust += 1;
                    }
                    telemetry.addData("dawinch power: ", "%f", dawinchAdjust);
                    telemetry.update();
                    changingDaWinchiSpeed = false;
                }
            }

            marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, gamepad2.right_stick_y * (dawinchAdjust / 10));
        }


        public void mrvSwitchtoGrabPosition () {
            if (gamepad2.a) {
                if (!assumingGrabPosition) {
                    timer_gp2_buttonA.reset();
                    assumingGrabPosition = true;
                } else if (timer_gp2_buttonA.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                    telemetry.addLine("GP2_A triggered. Will set Linac");
                    telemetry.update();

                    // Set Linac to Grab Position
                    marvyn.setRunMode(Mrv_Robot.MrvMotors.LIN_AC, STOP_AND_RESET_ENCODER);
                    marvyn.setRunMode(Mrv_Robot.MrvMotors.LIN_AC, RUN_WITHOUT_ENCODER);
                    marvyn.setTargetPosition(Mrv_Robot.MrvMotors.LIN_AC, Linac_Grab_Position_Ticks);
                    marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 1);
                    while (opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.LIN_AC) < Linac_Grab_Position_Ticks) {
                        idle();
                    }
                    marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 0);

                    // TODO: Set Dawinch to Grab Position
//                 marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, STOP_AND_RESET_ENCODER);
//                 marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, RUN_WITHOUT_ENCODER);
//                 marvyn.setTargetPosition(Mrv_Robot.MrvMotors.DA_WINCHI, Dawinchi_Grab_Position_Ticks);
//                 marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 1);
//                 while(opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.DA_WINCHI) < Dawinchi_Grab_Position_Ticks )
//                 {
//                     idle();
//                 }
//                 marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);

                    // Set Wrist Position parallel to ground
                    marvyn.Wristy.setPosition(Wrist_Parallel_to_Linac);
                    marvyn.The_Claw.setPosition(0);
                    assumingGrabPosition = false;
                }
            } else {
                assumingGrabPosition = false;
            }
        }

    }







