package msg_types;

public interface PositionArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "msg_types/PositionArray";
  static final java.lang.String _DEFINITION = "time stamp\nmsg_types/Position[] objects\n\n";
  org.ros.message.Time getStamp();
  void setStamp(org.ros.message.Time value);
  java.util.List<msg_types.Position> getObjects();
  void setObjects(java.util.List<msg_types.Position> value);
}
