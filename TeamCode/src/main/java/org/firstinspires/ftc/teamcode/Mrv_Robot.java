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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * ToDo: Vishruth/Satvika - please document what this class does
 */

public class Mrv_Robot
{
    enum MrvMotors
    {
        UPPER_LEFT,
        LOWER_LEFT,
        UPPER_RIGHT,
        LOWER_RIGHT,
        DUCK_WHEEL,
        LINEAR_ACTUATOR,
        WINCH,
        ALL_DRIVES,
        ALL_ATTACHMENTS,
        ALL
    }
    /* Public OpMode members. */
    public DcMotor upper_right = null;
    public DcMotor upper_left = null;
    public DcMotor lower_left = null;
    public DcMotor lower_right = null;
    public DcMotor duck_wheel = null;
    public DcMotor Linac_2 = null;
    public DcMotor Da_Winch = null;
    public Servo The_Claw = null;
    public Servo Wristy = null;
    Orientation angles;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Mrv_Robot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        upper_right  = hwMap.get(DcMotor.class, "Upper_Right");
        upper_left = hwMap.get(DcMotor.class, "Upper_Left");
        lower_left = hwMap.get(DcMotor.class, "Lower_Left");
        lower_right = hwMap.get(DcMotor.class, "Lower_Right");
        duck_wheel = hwMap.get(DcMotor.class, "Duck_Wheel");
        Linac_2 = hwMap.get(DcMotor.class, "Linac_2.0");
        Da_Winch = hwMap.get(DcMotor.class, "Da_Winch");


        //Servo
        The_Claw = hwMap.get(Servo.class, "The_Clawww");
        Wristy = hwMap.get(Servo.class, "Wristy");


        // Acquire gyro


        // Set all motors to zero power
        upper_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duck_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Linac_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Da_Winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        upper_left.setDirection(DcMotor.Direction.REVERSE);  //-
        upper_right.setDirection(DcMotor.Direction.FORWARD); //+
        lower_left.setDirection(DcMotor.Direction.REVERSE); //- used to be
        lower_right.setDirection(DcMotor.Direction.FORWARD); //+ used to be
        duck_wheel.setDirection(DcMotor.Direction.FORWARD);
        Linac_2.setDirection(DcMotor.Direction.FORWARD);
        Da_Winch.setDirection(DcMotor.Direction.FORWARD);


    }
    String formatAngle( AngleUnit angleUnit, double angle) {
        return formatDegrees(angleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    public void setRunMode(MrvMotors eWhichMotor, DcMotor.RunMode eMode )
    {

        switch (eWhichMotor){
            case UPPER_LEFT:
                upper_left.setMode(eMode);
                break;
            case UPPER_RIGHT:
                upper_right.setMode(eMode);
                break;
            case LOWER_LEFT:
                lower_left.setMode(eMode);
                break;
            case LOWER_RIGHT:
                lower_right.setMode(eMode);
                break;
            case DUCK_WHEEL:
                duck_wheel.setMode(eMode);
            case LINEAR_ACTUATOR:
                duck_wheel.setMode(eMode);
            case WINCH:
                duck_wheel.setMode(eMode);
            case ALL_DRIVES:
                lower_right.setMode(eMode);
                lower_left.setMode(eMode);
                upper_right.setMode(eMode);
                upper_left.setMode(eMode);
                break;
            case ALL_ATTACHMENTS:
                Linac_2.setMode(eMode);
                duck_wheel.setMode(eMode);
                Da_Winch.setMode(eMode);
            case ALL:
                lower_right.setMode(eMode);
                lower_left.setMode(eMode);
                upper_right.setMode(eMode);
                upper_left.setMode(eMode);
                duck_wheel.setMode(eMode);
                Da_Winch.setMode(eMode);
                Linac_2.setMode(eMode);


        }
    }

    public void setPower(MrvMotors eWhichMotor, double dPower )
    {

        switch (eWhichMotor){
            case UPPER_LEFT:
                upper_left.setPower(dPower);
                break;
            case UPPER_RIGHT:
                upper_right.setPower(dPower);
                break;
            case LOWER_LEFT:
                lower_left.setPower(dPower);
                break;
            case LOWER_RIGHT:
                lower_right.setPower(dPower);
                break;
            case DUCK_WHEEL:
                duck_wheel.setPower(dPower);
            case LINEAR_ACTUATOR:
                Linac_2.setPower(dPower);
            case WINCH:
                Da_Winch.setPower(dPower);
            case ALL_DRIVES:
                lower_right.setPower(dPower);
                lower_left.setPower(dPower);
                upper_right.setPower(dPower);
                upper_left.setPower(dPower);
                break;
            case ALL:
                lower_right.setPower(dPower);
                lower_left.setPower(dPower);
                upper_right.setPower(dPower);
                upper_left.setPower(dPower);
                duck_wheel.setPower(dPower);
                Da_Winch.setPower(dPower);
                Linac_2.setPower(dPower);

        }
    }

    public int getCurrentPosition( MrvMotors eWhichMotor )
    {
        switch(eWhichMotor)
        {
            case UPPER_LEFT:
                return upper_left.getCurrentPosition();
            case LOWER_LEFT:
                return lower_left.getCurrentPosition();
            case UPPER_RIGHT:
                return upper_right.getCurrentPosition();
            case LOWER_RIGHT:
                return lower_right.getCurrentPosition();
            case LINEAR_ACTUATOR:
                Linac_2.getCurrentPosition();
            case WINCH:
                Da_Winch.getCurrentPosition();
            case ALL_DRIVES:
                lower_right.getCurrentPosition();
                lower_left.getCurrentPosition();
                upper_right.getCurrentPosition();
                upper_left.getCurrentPosition();
            case ALL_ATTACHMENTS:
                duck_wheel.getCurrentPosition();
                Linac_2.getCurrentPosition();
                Da_Winch.getCurrentPosition();
            case ALL:
                lower_right.getCurrentPosition();
                lower_left.getCurrentPosition();
                upper_right.getCurrentPosition();
                upper_left.getCurrentPosition();
                duck_wheel.getCurrentPosition();
                Da_Winch.getCurrentPosition();
                Linac_2.getCurrentPosition();
            default:
                return 0;
        }
    }

    public void setTargetPosition( MrvMotors eWhichMotor, int iPos )
    {
        switch( eWhichMotor)
        {
            case UPPER_LEFT:
                upper_left.setTargetPosition(iPos);
                break;
            case LOWER_LEFT:
                lower_left.setTargetPosition(iPos);
                break;
            case UPPER_RIGHT:
                upper_right.setTargetPosition(iPos);
                break;
            case LOWER_RIGHT:
                lower_right.setTargetPosition(iPos);
                break;
            case LINEAR_ACTUATOR:
                Linac_2.setTargetPosition(iPos);
            case WINCH:
                Da_Winch.setTargetPosition(iPos);
            case ALL_DRIVES:
                lower_right.setTargetPosition(iPos);
                lower_left.setTargetPosition(iPos);
                upper_right.setTargetPosition(iPos);
                upper_left.setTargetPosition(iPos);
                break;
            case ALL:
                lower_right.setTargetPosition(iPos);
                lower_left.setTargetPosition(iPos);
                upper_right.setTargetPosition(iPos);
                upper_left.setTargetPosition(iPos);
                duck_wheel.setTargetPosition(iPos);
                Da_Winch.setTargetPosition(iPos);
                Linac_2.setTargetPosition(iPos);
            default :
                break;
        }
    }

    public boolean areMotorsBusy(MrvMotors eWhichMotor) {

        switch(eWhichMotor)
        {
            case UPPER_LEFT: // upper left
                return upper_left.isBusy();
            case LOWER_LEFT: // lower left
                return lower_left.isBusy();
            case UPPER_RIGHT: // upper right
                return upper_right.isBusy();
            case LOWER_RIGHT: // lower right
                return lower_right.isBusy();
            case DUCK_WHEEL:
                return duck_wheel.isBusy();
            case LINEAR_ACTUATOR:
                return Linac_2.isBusy();
            case WINCH:
                return Da_Winch.isBusy();
            case ALL_DRIVES: // All Drives
                return lower_left.isBusy() && lower_right.isBusy() && upper_left.isBusy() && upper_right.isBusy();
            case ALL_ATTACHMENTS:
                return Linac_2.isBusy() && duck_wheel.isBusy() && Da_Winch.isBusy();
            case ALL:
                return lower_left.isBusy() && lower_right.isBusy() && upper_left.isBusy() && upper_right.isBusy() && duck_wheel.isBusy() && Linac_2.isBusy()&& Da_Winch.isBusy();
            default:
                return false;
        }
    }

 }

