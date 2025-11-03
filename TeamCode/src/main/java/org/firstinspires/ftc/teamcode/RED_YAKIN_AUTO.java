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

@Autonomous(name = "RED YAKIN AUTO")
public class RED_YAKIN_AUTO extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(33.50, 135.50, Math.toRadians(180));
    private final Pose scorePose = new Pose(65.00, 84.00, Math.toRadians(120));
    private final Pose score2Pose = new Pose(65.00, 84.00, Math.toRadians(135));
    private final Pose score3Pose = new Pose(57.00, 108.00, Math.toRadians(135));
    private final Pose pickUp1Pose = new Pose(19.00, 84.00, Math.toRadians(180));
    private final Pose pickUp2Pose = new Pose(24.00, 60.00, Math.toRadians(180));
    private final Pose pickUpControl2Pose = new Pose(79.00, 55.00, Math.toRadians(180));


    private Path scorePreload;
    private PathChain pickUp1, scorePickUp1, pickUp2, scorePickUp2;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pickUp1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,pickUp1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickUp1Pose.getHeading())
                .build();

        scorePickUp1 = follower.pathBuilder()
                .addPath(new BezierLine(pickUp1Pose,score2Pose))
                .setLinearHeadingInterpolation(pickUp1Pose.getHeading(),score2Pose.getHeading())
                .build();

        pickUp2 = follower.pathBuilder()
                .addPath(new BezierCurve(score2Pose,pickUpControl2Pose,pickUp2Pose))
                .setLinearHeadingInterpolation(score2Pose.getHeading(),pickUp2Pose.getHeading())
                .build();

        scorePickUp2 = follower.pathBuilder()
                .addPath(new BezierLine(pickUp2Pose,score3Pose))
                .setLinearHeadingInterpolation(pickUp2Pose.getHeading(),score3Pose.getHeading())
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
                    follower.followPath(pickUp1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    follower.followPath(scorePickUp1,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    Superstructure.setCurrentWantedState(Superstructure.wantedState.IDLE);
                    setPathState(5);
                }
                break;
            case 5:
                Superstructure.setCurrentWantedState(Superstructure.wantedState.SHOOT);
                setPathState(6);
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 5.0){
                    Superstructure.setCurrentWantedState(Superstructure.wantedState.INTAKE);
                    follower.followPath(pickUp2,true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    follower.followPath(scorePickUp2,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    Superstructure.setCurrentWantedState(Superstructure.wantedState.IDLE);
                    setPathState(9);
                }
                break;
            case 9:
                Superstructure.setCurrentWantedState(Superstructure.wantedState.HOME);
                setPathState(-1);
//                Superstructure.setCurrentWantedState(Superstructure.wantedState.SHOOT);
//                setPathState(10);
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        Superstructure.setIsBlue(false);
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
