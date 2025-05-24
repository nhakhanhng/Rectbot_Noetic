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
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class KeyPoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.point = null;
      this.score = null;
    }
    else {
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = new geometry_msgs.msg.Point32();
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
    // Serializes a message object of type KeyPoints
    // Serialize message field [point]
    bufferOffset = geometry_msgs.msg.Point32.serialize(obj.point, buffer, bufferOffset);
    // Serialize message field [score]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.score, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type KeyPoints
    let len;
    let data = new KeyPoints(null);
    // Deserialize message field [point]
    data.point = geometry_msgs.msg.Point32.deserialize(buffer, bufferOffset);
    // Deserialize message field [score]
    data.score = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rectbot_cv/KeyPoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '92f94618554fe5449a29ca0200149d32';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point32 point
    std_msgs/Float32 score
    ================================================================================
    MSG: geometry_msgs/Point32
    # This contains the position of a point in free space(with 32 bits of precision).
    # It is recommeded to use Point wherever possible instead of Point32.  
    # 
    # This recommendation is to promote interoperability.  
    #
    # This message is designed to take up less space when sending
    # lots of points at once, as in the case of a PointCloud.  
    
    float32 x
    float32 y
    float32 z
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
    const resolved = new KeyPoints(null);
    if (msg.point !== undefined) {
      resolved.point = geometry_msgs.msg.Point32.Resolve(msg.point)
    }
    else {
      resolved.point = new geometry_msgs.msg.Point32()
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

module.exports = KeyPoints;
