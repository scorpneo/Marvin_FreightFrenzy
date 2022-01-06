package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive")
public class Mrv_Autonomous extends LinearOpMode {
    Mrv_Robot marvyn = new Mrv_Robot();
    public static double x = 3;
    public static double y = -63;
    public static double heading = 90;
    public static Pose2d TargetPos = new Pose2d(-12,-48, Math.toRadians(90));
    public static Vector2d SplineVec = new Vector2d(-48,-56);
    public static double SplineHeading = -135;

    @Override
    public void runOpMode() throws InterruptedException {
      marvyn.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = marvyn.mecanumDrive.trajectorySequenceBuilder(new Pose2d(x,y,Math.toRadians(heading)))
                //drive to hub
                .lineToLinearHeading(TargetPos)
                //TODO: set exact end pose for duck spline
                .splineTo(SplineVec, Math.toRadians(SplineHeading))
                //do duck
                .waitSeconds(3)
                //go to warehouse entrance
                //.lineToLinearHeading(new Pose2d(18,-64,Math.toRadians(0)))
                //enter warehouse
                //.forward(36)
                //wait to not cause the weird Roadrunner heading issue
                //.waitSeconds(0.5)
                //move over
                //.lineToLinearHeading(new Pose2d(56,-40,Math.toRadians(-90)))
                //.strafeLeft(24)

                .build()
        ;


        marvyn.mecanumDrive.followTrajectorySequence(traj);


    }
}


