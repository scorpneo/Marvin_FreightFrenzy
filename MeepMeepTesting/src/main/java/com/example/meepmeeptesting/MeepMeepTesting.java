package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String args[]) {
        MeepMeep mm = new MeepMeep(1000);

         Pose2d blue_1_pose_estimate      = new Pose2d(-33.75, 62.625, Math.toRadians(-90));
         Pose2d red_1_pose_estimate       = new Pose2d(-37, -62.625, Math.toRadians(90));

         Pose2d blue_2_pose_estimate      = new Pose2d( 13, 62.625, Math.toRadians(-90));
         Pose2d red_2_pose_estimate       = new Pose2d( 9.75, -62.625, Math.toRadians(90));

         Pose2d blue_1_shipping_hub_pos = new Pose2d(-21.5,  45.75, Math.toRadians(-70));
         Pose2d red1_shipping_hub_pos      = new Pose2d(-21.5, -45.75, Math.toRadians(70));

         Pose2d blue_2_shipping_hub_pos     = new Pose2d(-1.625,  45.75, Math.toRadians(-110));
         Pose2d red2_shipping_hub_pos      = new Pose2d(-1.625, -45.75, Math.toRadians(110));

         Pose2d blue_duck_wheel_pos       = new Pose2d(-59.75, 55, Math.toRadians(0));
         Pose2d red_duck_wheel_pos        = new Pose2d(-59.75, -55, Math.toRadians(0));

         Pose2d blue_warehouse_enter_pos  = new Pose2d( 11, 64.75, Math.toRadians(0));
         Pose2d red_warehouse_enter_pos   = new Pose2d( 11, -64.75, Math.toRadians(0));

         Pose2d blue_warehouse_pos        = new Pose2d( 36.75, 64.75, Math.toRadians(0 ));
         Pose2d red_warehouse_pos         = new Pose2d( 36.75, -64.75, Math.toRadians(0));

         Pose2d blue_storage_pos          = new Pose2d( -59.75, 35.5, Math.toRadians(0));
         Pose2d red_storage_pos          = new Pose2d( -59.75, -35.5, Math.toRadians(0));

         Pose2d blue_park_pos             = new Pose2d( 64, 36, Math.toRadians(90));
         Pose2d red_park_pos              = new Pose2d( 64, -36, Math.toRadians(-90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 11.8)
                .followTrajectorySequence(drive ->
                        //red duck
                        drive.trajectorySequenceBuilder(blue_2_pose_estimate)
                                //target pos
                                .lineToLinearHeading(blue_2_shipping_hub_pos)
                                .lineToLinearHeading(blue_warehouse_enter_pos)
                                //go to warehouse entrance
                                .lineToLinearHeading(blue_warehouse_pos)
                                .lineToLinearHeading(blue_park_pos)
                                //enter warehouse
                                .build()
                );

        mm.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}