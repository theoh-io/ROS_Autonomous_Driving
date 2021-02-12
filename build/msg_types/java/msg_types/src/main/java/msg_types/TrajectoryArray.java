package msg_types;

public interface TrajectoryArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "msg_types/TrajectoryArray";
  static final java.lang.String _DEFINITION = "time stamp\nmsg_types/PositionArray[] trajectories\n\n";
  org.ros.message.Time getStamp();
  void setStamp(org.ros.message.Time value);
  java.util.List<msg_types.PositionArray> getTrajectories();
  void setTrajectories(java.util.List<msg_types.PositionArray> value);
}
