// Auto-generated. Do not edit!

// (in-package rectbot_cv.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class KeyPoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.score = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('score')) {
        this.score = initObj.score
      }
      else {
        this.score = new std_msgs.msg.Float32();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type KeyPoint
    // Serialize message field [x]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.y, buffer, bufferOffset);
    // Serialize message field [score]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.score, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type KeyPoint
    let len;
    let data = new KeyPoint(null);
    // Deserialize message field [x]
    data.x = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [score]
    data.score = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rectbot_cv/KeyPoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1cd825e8df87066e1486d6a8d6cfd264';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float32 x
    std_msgs/Float32 y
    std_msgs/Float32 score
    ================================================================================
    MSG: std_msgs/Float32
    float32 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new KeyPoint(null);
    if (msg.x !== undefined) {
      resolved.x = std_msgs.msg.Float32.Resolve(msg.x)
    }
    else {
      resolved.x = new std_msgs.msg.Float32()
    }

    if (msg.y !== undefined) {
      resolved.y = std_msgs.msg.Float32.Resolve(msg.y)
    }
    else {
      resolved.y = new std_msgs.msg.Float32()
    }

    if (msg.score !== undefined) {
      resolved.score = std_msgs.msg.Float32.Resolve(msg.score)
    }
    else {
      resolved.score = new std_msgs.msg.Float32()
    }

    return resolved;
    }
};

module.exports = KeyPoint;
