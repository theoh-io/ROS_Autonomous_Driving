// Auto-generated. Do not edit!

// (in-package msg_types.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let State = require('./State.js');
let TrajectoryArray = require('./TrajectoryArray.js');

//-----------------------------------------------------------

class StateArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.desired_path = null;
      this.sync_predictions = null;
      this.initial_state = null;
    }
    else {
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('desired_path')) {
        this.desired_path = initObj.desired_path
      }
      else {
        this.desired_path = [];
      }
      if (initObj.hasOwnProperty('sync_predictions')) {
        this.sync_predictions = initObj.sync_predictions
      }
      else {
        this.sync_predictions = new TrajectoryArray();
      }
      if (initObj.hasOwnProperty('initial_state')) {
        this.initial_state = initObj.initial_state
      }
      else {
        this.initial_state = new State();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StateArray
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [desired_path]
    // Serialize the length for message field [desired_path]
    bufferOffset = _serializer.uint32(obj.desired_path.length, buffer, bufferOffset);
    obj.desired_path.forEach((val) => {
      bufferOffset = State.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [sync_predictions]
    bufferOffset = TrajectoryArray.serialize(obj.sync_predictions, buffer, bufferOffset);
    // Serialize message field [initial_state]
    bufferOffset = State.serialize(obj.initial_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StateArray
    let len;
    let data = new StateArray(null);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [desired_path]
    // Deserialize array length for message field [desired_path]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.desired_path = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.desired_path[i] = State.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [sync_predictions]
    data.sync_predictions = TrajectoryArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [initial_state]
    data.initial_state = State.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 28 * object.desired_path.length;
    length += TrajectoryArray.getMessageSize(object.sync_predictions);
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'msg_types/StateArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f6495b47ab8804317c3ad5428e9da0a5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time stamp
    msg_types/State[] desired_path
    msg_types/TrajectoryArray sync_predictions
    msg_types/State initial_state
    
    
    ================================================================================
    MSG: msg_types/State
    time stamp
    float32 x
    float32 y
    float32 heading
    float32 v
    float32 w
    
    
    ================================================================================
    MSG: msg_types/TrajectoryArray
    time stamp
    msg_types/PositionArray[] trajectories
    
    
    ================================================================================
    MSG: msg_types/PositionArray
    time stamp
    msg_types/Position[] objects
    
    
    ================================================================================
    MSG: msg_types/Position
    time stamp
    float32 x
    float32 y
    float32 t
    uint32 id
    bool actual
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StateArray(null);
    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    if (msg.desired_path !== undefined) {
      resolved.desired_path = new Array(msg.desired_path.length);
      for (let i = 0; i < resolved.desired_path.length; ++i) {
        resolved.desired_path[i] = State.Resolve(msg.desired_path[i]);
      }
    }
    else {
      resolved.desired_path = []
    }

    if (msg.sync_predictions !== undefined) {
      resolved.sync_predictions = TrajectoryArray.Resolve(msg.sync_predictions)
    }
    else {
      resolved.sync_predictions = new TrajectoryArray()
    }

    if (msg.initial_state !== undefined) {
      resolved.initial_state = State.Resolve(msg.initial_state)
    }
    else {
      resolved.initial_state = new State()
    }

    return resolved;
    }
};

module.exports = StateArray;
