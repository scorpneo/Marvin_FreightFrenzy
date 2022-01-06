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

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                        //red duck
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(-12,-48,Math.toRadians(90)))
                                //spline to duck wheel
                                //TODO: set exact end pose for duck spline
                                .splineTo(new Vector2d(-48,-56),Math.toRadians(-135))
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
                );

        mm.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}