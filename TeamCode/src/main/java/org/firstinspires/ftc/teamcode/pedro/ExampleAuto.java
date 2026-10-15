package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous
public class ExampleAuto extends OpMode {
    private Follower follower; // Add this
    private final PoseFactory p = PoseFactory.degrees(); // add this
    // our poses
    private final Pose startPose = p.of(84, 133, 270);
    private final Pose park = p.of(84, 120, 270);
    private Path park() {
        return line(startPose, park).linear(startPose, park);
    }
    private final Pose controlPose = p.of(36, 60, 45);
    private Path parkCurvy() {
        return curve(startPose, controlPose, park).linear(startPose, park);
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        schedule(follow(follower, park()));
        // using the same park path from the last page
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

    }
}