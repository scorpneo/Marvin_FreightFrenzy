package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive")
public class Mrv_Autonomous extends LinearOpMode {
    Mrv_Robot marvyn = new Mrv_Robot();
    public static double starting_pos_x = 12;

    public static double starting_pos_y = -62;
    public static double starting_pos_heading = 90;
    public static double target_pos_x = 0;
    public static double target_pos_y = -48;
    public static double target_pos_heading = 130;
    public static double spline_x = -55;
    public static double spline_y = -55;
    public static double spline_heading = 90;
    public static double enter_x = 24;
    public static double enter_y = -64;
    public static double enter_heading = 0;
    public static double warehouse_x = 40;
    public static double warehouse_y = -64;
    public static double warehouse_heading = 0;
    public static double park_x = 64;
    public static double park_y = -36;
    public static double park_heading = -90;

//    public static Pose2d TargetPos = new Pose2d(-12,-48, Math.toRadians(90));
//    public static Vector2d SplineVec = new Vector2d(-48,-56);

    @Override
    public void runOpMode() throws InterruptedException {
      marvyn.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d StartingPos = new Pose2d(starting_pos_x, starting_pos_y, Math.toRadians(starting_pos_heading));
        Pose2d TargetPos = new Pose2d(target_pos_x, target_pos_y, Math.toRadians(target_pos_heading));
        Pose2d lineVec = new Pose2d(spline_x, spline_y, Math.toRadians(spline_heading));
        Pose2d enterPos = new Pose2d(enter_x, enter_y, Math.toRadians(enter_heading));
        Pose2d warehousePos = new Pose2d(warehouse_x, warehouse_y, Math.toRadians(warehouse_heading));
        Pose2d parkPos = new Pose2d(park_x, park_y, Math.toRadians(park_heading));

        marvyn.mecanumDrive.setPoseEstimate(StartingPos);

        TrajectorySequence Red2 = marvyn.mecanumDrive.trajectorySequenceBuilder(new Pose2d (12,-62,Math.toRadians(90)))
                //drive to hub
                .lineToLinearHeading(new Pose2d(0,-48,Math.toRadians(130)))
                //TODO: set exact end pose for duck spline
                //go to entrance
                .lineToLinearHeading(new Pose2d(24,-64,Math.toRadians(0)))
                //enter warehouse
                .lineToLinearHeading(new Pose2d(40,-64,Math.toRadians(0)))
                //park
                .lineToLinearHeading(new Pose2d(64,-36,Math.toRadians(90)))

                .build()
                ;

        TrajectorySequence Blue2 = marvyn.mecanumDrive.trajectorySequenceBuilder(new Pose2d (12,-62,Math.toRadians(90)))
                //drive to hub
                .lineToLinearHeading(new Pose2d(0,48,Math.toRadians(-130)))
                //TODO: set exact end pose for duck spline
                //go to entrance
                .lineToLinearHeading(new Pose2d(24,64,Math.toRadians(0)))
                //enter warehouse
                .lineToLinearHeading(new Pose2d(40,64,Math.toRadians(0)))
                //park
                .lineToLinearHeading(new Pose2d(64,36,Math.toRadians(-90)))

                .build()
                ;

        TrajectorySequence Red1 = marvyn.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-36,-62,Math.toRadians(90)))
                //drive to hub
                .lineToLinearHeading(new Pose2d(-12,-48,Math.toRadians(40)))
                //TODO: set exact end pose for duck spline
                .lineToLinearHeading(new Pose2d(-55,-55,Math.toRadians(90)))
                //do duck
                .waitSeconds(3)
                //go to warehouse entrance
                .lineToLinearHeading(new Pose2d(24,-64,Math.toRadians(0)))
                //enter warehouse
                .lineToLinearHeading(new Pose2d(40,-64,Math.toRadians(0)))
                //park
                .lineToLinearHeading(new Pose2d(64,-36,Math.toRadians(90)))

                .build()
                ;

        TrajectorySequence Blue1 = marvyn.mecanumDrive.trajectorySequenceBuilder(new Pose2d(-36,-62,Math.toRadians(90)))
                //drive to hub
                .lineToLinearHeading(new Pose2d(-12,48,Math.toRadians(-40)))
                //TODO: set exact end pose for duck spline
                .lineToLinearHeading(new Pose2d(-55,55,Math.toRadians(-90)))
                //do duck
                .waitSeconds(3)
                //go to warehouse entrance
                .lineToLinearHeading(new Pose2d(24,64,Math.toRadians(0)))
                //enter warehouse
                .lineToLinearHeading(new Pose2d(40,64,Math.toRadians(0)))
                //park
                .lineToLinearHeading(new Pose2d(64,36,Math.toRadians(-90)))

                .build()
                ;

        TrajectorySequence traj = marvyn.mecanumDrive.trajectorySequenceBuilder(StartingPos)
                //drive to hub
                .lineToLinearHeading(TargetPos)
                //TODO: set exact end pose for duck spline
                .lineToLinearHeading(lineVec)
                //do duck
                .waitSeconds(3)
                //go to warehouse entrance
                .lineToLinearHeading(enterPos)
                //enter warehouse
                .lineToLinearHeading(warehousePos)
                //park
                .lineToLinearHeading(parkPos)

                .build()
        ;


        marvyn.mecanumDrive.followTrajectorySequence(traj);
    }
}


