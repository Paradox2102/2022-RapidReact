package frc.robot;

import frc.lib.Logger;
import frc.robot.PositionTracker.PositionContainer;

public class Navigator {
    PositionTracker m_tracker;
    Object m_lock = new Object();

    public Navigator(PositionTracker pos) {
        m_tracker = pos;
    }

    public class NavigatorPos {
        public final double yaw;
        public final double x;
        public final double y;
        public final double leftPos;
        public final double rightPos;
        public final float leftVel;
        public final float rightVel;

        private NavigatorPos(double yaw, double x, double y, double leftPos, double rightPos, float leftVel, float rightVel) {
            this.yaw = yaw;
            this.x = x;
            this.y = y;
            this.leftPos = leftPos;
            this.rightPos = rightPos;
            this.leftVel = leftVel;
            this.rightVel = rightVel;
        }
    }

    public void reset(double angle, double x, double y) {
        Logger.Log("Navigator", 1, String.format("Reset:a=%f,x=%f,y=%f", angle, x, y));
        m_tracker.setAngle(angle);
        m_tracker.setXY(x, y);
    }

    public NavigatorPos getPos() {
        NavigatorPos pos;
        PositionContainer con = m_tracker.getPos();
        synchronized(m_lock) {
            pos = new NavigatorPos(m_tracker.getAngle(), con.x, con.y, m_tracker.getLeftEncoderPos(), m_tracker.getRightEncoderPos(), (float) m_tracker.getLeftEncoderVel(), (float)m_tracker.getRightEncoderVel());
        }
        return pos;
    }
}