// Auto-generated. Do not edit!

// (in-package robotics_project.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let ObjectPoseArray = require('../msg/ObjectPoseArray.js');

//-----------------------------------------------------------

class VisionResultsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VisionResultsRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VisionResultsRequest
    let len;
    let data = new VisionResultsRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robotics_project/VisionResultsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VisionResultsRequest(null);
    return resolved;
    }
};

class VisionResultsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = new ObjectPoseArray();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VisionResultsResponse
    // Serialize message field [x]
    bufferOffset = ObjectPoseArray.serialize(obj.x, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VisionResultsResponse
    let len;
    let data = new VisionResultsResponse(null);
    // Deserialize message field [x]
    data.x = ObjectPoseArray.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ObjectPoseArray.getMessageSize(object.x);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robotics_project/VisionResultsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '08490d80b3f7601e8e2c1c05e10bad72';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ObjectPoseArray x
    
    
    ================================================================================
    MSG: robotics_project/ObjectPoseArray
    ObjectPose[] poses
    
    ================================================================================
    MSG: robotics_project/ObjectPose
    string name
    geometry_msgs/Pose2D pose
    int8 face
    
    ================================================================================
    MSG: geometry_msgs/Pose2D
    # Deprecated
    # Please use the full 3D pose.
    
    # In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.
    
    # If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.
    
    
    # This expresses a position and orientation on a 2D manifold.
    
    float64 x
    float64 y
    float64 theta
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VisionResultsResponse(null);
    if (msg.x !== undefined) {
      resolved.x = ObjectPoseArray.Resolve(msg.x)
    }
    else {
      resolved.x = new ObjectPoseArray()
    }

    return resolved;
    }
};

module.exports = {
  Request: VisionResultsRequest,
  Response: VisionResultsResponse,
  md5sum() { return '08490d80b3f7601e8e2c1c05e10bad72'; },
  datatype() { return 'robotics_project/VisionResults'; }
};
