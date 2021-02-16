package msg_types;

public interface StateArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "msg_types/StateArray";
  static final java.lang.String _DEFINITION = "time stamp\nmsg_types/State[] desired_path\nmsg_types/TrajectoryArray sync_predictions\nmsg_types/State initial_state\n\n";
  org.ros.message.Time getStamp();
  void setStamp(org.ros.message.Time value);
  java.util.List<msg_types.State> getDesiredPath();
  void setDesiredPath(java.util.List<msg_types.State> value);
  msg_types.TrajectoryArray getSyncPredictions();
  void setSyncPredictions(msg_types.TrajectoryArray value);
  msg_types.State getInitialState();
  void setInitialState(msg_types.State value);
}
