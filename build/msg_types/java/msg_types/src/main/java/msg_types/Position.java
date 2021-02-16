package msg_types;

public interface Position extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "msg_types/Position";
  static final java.lang.String _DEFINITION = "time stamp\nfloat32 x\nfloat32 y\nfloat32 t\nuint32 id\nbool actual\n";
  org.ros.message.Time getStamp();
  void setStamp(org.ros.message.Time value);
  float getX();
  void setX(float value);
  float getY();
  void setY(float value);
  float getT();
  void setT(float value);
  int getId();
  void setId(int value);
  boolean getActual();
  void setActual(boolean value);
}
