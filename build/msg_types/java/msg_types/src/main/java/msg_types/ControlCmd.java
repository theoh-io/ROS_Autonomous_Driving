package msg_types;

public interface ControlCmd extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "msg_types/ControlCmd";
  static final java.lang.String _DEFINITION = "Header header\nfloat32 v\nfloat32 w\n\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float getV();
  void setV(float value);
  float getW();
  void setW(float value);
}
