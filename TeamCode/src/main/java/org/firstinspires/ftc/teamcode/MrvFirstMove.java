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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;


import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous
public class MrvFirstMove extends LinearOpMode {

    public static double xPos = 30;
    public static double yPos = 30;
    public static double turnPos = 0;
    public static double speed = 15;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive MrvDrive = new SampleMecanumDrive(hardwareMap);

        // Steps for Robot Autonomous execution
        // 1. Use Vuforia to detect starting position
        // 2. Map Vuforia given field coordinates to Tensorflow screen coordinates
        waitForStart();
        // 3. Use Tensorflow to read barcode and identify duck position
        // 4. Generate trajectory to drop off freight drop off
        // 5. Follow trajectory
        // 6. Freight drop off
        // 7. Generate trajectory to go to duck wheel (red/blue)
        // 8. Follow trajectory
        // 9. Spin duck wheel
        // 10. Generate trajectory to park in warehouse
        // 11. Follow trajectory to park in warehouse

        // Alternately
        //  7. Generate trajectory to go to warehouse
        //  8. Use Tensorflow to detect freight element in warehouse
        //  9. Follow trajectory
        //  10. pick up freight element
        //  11. Generate trajectory to return to freight drop off
        //  12. Follow trajectory
        //  13. Freight Drop off
        //  14. Generate trajectory to park in Warehouse
        //  15. Follow trajectory to park in Warehouse

        if (isStopRequested()) return;

        //Building the trajectory
        Trajectory MrvFirstMove = MrvDrive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(xPos,yPos),Math.toRadians(turnPos),
                    SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        MrvDrive.followTrajectory(MrvFirstMove);

        sleep(2000);

        MrvDrive.followTrajectory(MrvDrive.trajectoryBuilder(MrvFirstMove.end(), true)
        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
        .build());
    };

    }


