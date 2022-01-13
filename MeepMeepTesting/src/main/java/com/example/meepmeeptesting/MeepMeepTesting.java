package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String args[]) {
        MeepMeep mm = new MeepMeep(800);

        Pose2d blue_1_pose_estimate      = new Pose2d( -36, 64, Math.toRadians(-90));

        Pose2d blue_shipping_hub_pos     = new Pose2d(-12,  48, Math.toRadians(-90));
        Pose2d blue_duck_wheel_pos       = new Pose2d(-55, 55, Math.toRadians(-45));
        Pose2d blue_warehouse_enter_pos  = new Pose2d( 24, 64, Math.toRadians(0));
        Pose2d blue_warehouse_pos        = new Pose2d( 40, 64, Math.toRadians(0 ));
        Pose2d blue_park_pos             = new Pose2d( 64, 36, Math.toRadians(90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                        //red duck
                        drive.trajectorySequenceBuilder(blue_1_pose_estimate)
                                //target pos
                                .lineToLinearHeading(blue_shipping_hub_pos)

                                .lineToLinearHeading(blue_duck_wheel_pos)
                                //do duck
                                //go to warehouse entrance
                                .lineToLinearHeading(blue_warehouse_enter_pos)
                                //enter warehouse
                                .lineToLinearHeading(blue_warehouse_pos)
                                //park
                                .lineToLinearHeading(blue_park_pos)
                                .build()
                );

        mm.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}