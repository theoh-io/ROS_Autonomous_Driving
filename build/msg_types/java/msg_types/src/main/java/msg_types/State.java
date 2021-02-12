package msg_types;

public interface State extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "msg_types/State";
  static final java.lang.String _DEFINITION = "time stamp\nfloat32 x\nfloat32 y\nfloat32 heading\nfloat32 v\nfloat32 w\n\n";
  org.ros.message.Time getStamp();
  void setStamp(org.ros.message.Time value);
  float getX();
  void setX(float value);
  float getY();
  void setY(float value);
  float getHeading();
  void setHeading(float value);
  float getV();
  void setV(float value);
  float getW();
  void setW(float value);
}
