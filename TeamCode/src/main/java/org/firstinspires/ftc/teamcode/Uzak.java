//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.lib.Subsystems.Superstructure;
//
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//
//
//@Autonomous(name = "BUCKET 4 🥵")
//public class Uzak extends CommandOpMode {
//    private double loopTime = 0.0;
//
//    private boolean flag = true;
//
//
//    @Override
//    public void initialize() {
//        CommandScheduler.getInstance().reset();
//
//
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
//        Superstructure.init(hardwareMap);
//
//        Superstructure.read();
//        Superstructure.drivetrain.OdometryModule.resetPosAndIMU();
//        Superstructure.drivetrain.OdometryModule.resetPose(new Pose2d(0,0,new Rotation2d()));
//
//        while (!isStarted()) {
//            telemetry.addLine("auto in init");
//            telemetry.update();
//        }
//
//        schedule(
//                new SequentialCommandGroup(
//                        new PositionCommand(new Pose2d(0.6,0.2,Rotation2d.fromDegrees(0)),1000),
//                        new InstantCommand(()->Superstructure.setCurrentWantedState(Superstructure.wantedState.SHOOT)),
//                        new WaitUntilCommand(()->!Superstructure.revolver.RevolverHasBall()&&Superstructure.currentSystemState== Superstructure.systemState.AIMING),
//                        new InstantCommand(()->Superstructure.setCurrentWantedState(Superstructure.wantedState.INTAKE)),
//                        new PositionCommand(new Pose2d(0.8,-0.4,Rotation2d.fromDegrees(90)),3000),
//                        new PositionCommand(new Pose2d(0.6,0.2,Rotation2d.fromDegrees(0)),1000),
//                        new InstantCommand(()->Superstructure.setCurrentWantedState(Superstructure.wantedState.SHOOT)),
//                        new WaitUntilCommand(()->!Superstructure.revolver.RevolverHasBall()&&Superstructure.currentSystemState== Superstructure.systemState.AIMING),
//                        new InstantCommand(()->Superstructure.setCurrentWantedState(Superstructure.wantedState.INTAKE)),
//                        new PositionCommand(new Pose2d(1.4,-0.4,Rotation2d.fromDegrees(90)),1000),
//                        new PositionCommand(new Pose2d(1.8,0.2,Rotation2d.fromDegrees(37)),1000),
//                        new InstantCommand(()->Superstructure.setCurrentWantedState(Superstructure.wantedState.SHOOT)),
//                        new WaitUntilCommand(()->!Superstructure.revolver.RevolverHasBall()&&Superstructure.currentSystemState== Superstructure.systemState.AIMING),
//                        new InstantCommand(()->Superstructure.setCurrentWantedState(Superstructure.wantedState.INTAKE)),
//                        new PositionCommand(new Pose2d(2,-0.4,Rotation2d.fromDegrees(90)),1000),
//                        new PositionCommand(new Pose2d(1.8,0.2,Rotation2d.fromDegrees(37)),1000),
//                        new InstantCommand(()->Superstructure.setCurrentWantedState(Superstructure.wantedState.SHOOT)),
//                        new WaitUntilCommand(()->!Superstructure.revolver.RevolverHasBall()&&Superstructure.currentSystemState== Superstructure.systemState.AIMING)
//
//                        ));
//    }
//
//    @Override
//    public void run() {
//        Superstructure.read();
//
//        super.run();
//        Superstructure.periodic();
//
//        Pose2d currentPose = Superstructure.drivetrain.OdometryModule.getPose2d();
//
//
//        double loop = System.nanoTime();
//        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//        loopTime = loop;
//        telemetry.addData("three", currentPose.toString());
////        telemetry.addData("two", localizer.getPose().toString());
//
//        telemetry.addData("X", Superstructure.drivetrain.OdometryModule.getPose2d().getX());
//        telemetry.addData("Y", Superstructure.drivetrain.OdometryModule.getPose2d().getY());
//        telemetry.update();
//
////        if (gamepad1.a && flag) {
////            CommandScheduler.getInstance().schedule(new RelocalizeCommand());
////            flag = false;
//////            localizer.setPose(globalTagPosition);
////        }
////
//        Superstructure.write();
////        robot.clearBulkCache();
//
////        if (isStopRequested()) robot.closeCamera();
//    }
//}