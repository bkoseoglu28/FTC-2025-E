package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Yakın Auto")

public class YAKIN extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(63.00, 135.00, Math.toRadians(180)); // Start Pose of our robot.

    private final Pose scorePose = new Pose(63.00, 84.00, Math.toRadians(180));

    private final Pose pickUpPose = new Pose(16.00, 84.00, Math.toRadians(180));


    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,pickUpPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickUpPose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose,scorePose))
                .setLinearHeadingInterpolation(pickUpPose.getHeading(),scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    Superstructure.setCurrentWantedState(Superstructure.wantedState.SHOOT);
                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 5.0){
                    Superstructure.setCurrentWantedState(Superstructure.wantedState.INTAKE);
                    follower.followPath(grabPickup1,true);  // Path'i burada başlat
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    follower.followPath(scorePickup1,true);
                    setPathState(4);
                }
                break;
            case 4:
                    if(!follower.isBusy()){
                        Superstructure.setCurrentWantedState(Superstructure.wantedState.SHOOT);
                        setPathState(5);
                    }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 5.0){  // Shooting için 5 saniye bekle
                    Superstructure.setCurrentWantedState(Superstructure.wantedState.IDLE);
                    setPathState(-1);  // Bitir
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        Superstructure.init(hardwareMap);
    }

    @Override
    public void loop() {
        Superstructure.read();
        Superstructure.periodic();
        Superstructure.write();
        Superstructure.setAngularVel(follower.getAngularVelocity());
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
