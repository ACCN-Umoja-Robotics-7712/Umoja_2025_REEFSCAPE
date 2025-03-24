package frc.robot.objects;

import edu.wpi.first.math.geometry.Pose2d;

public class Reef {
    public Pose2d center, leftBranch, rightBranch;
    
    public Reef(Pose2d center, Pose2d leftBranch, Pose2d rightBranch) {
        this.center = center;
        this.leftBranch = leftBranch;
        this.rightBranch = rightBranch;
    }
}
