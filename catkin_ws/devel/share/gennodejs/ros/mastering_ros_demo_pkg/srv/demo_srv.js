// Auto-generated. Do not edit!

// (in-package mastering_ros_demo_pkg.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class demo_srvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.in = null;
      this.out = null;
    }
    else {
      if (initObj.hasOwnProperty('in')) {
        this.in = initObj.in
      }
      else {
        this.in = '';
      }
      if (initObj.hasOwnProperty('out')) {
        this.out = initObj.out
      }
      else {
        this.out = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type demo_srvRequest
    // Serialize message field [in]
    bufferOffset = _serializer.string(obj.in, buffer, bufferOffset);
    // Serialize message field [out]
    bufferOffset = _serializer.string(obj.out, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type demo_srvRequest
    let len;
    let data = new demo_srvRequest(null);
    // Deserialize message field [in]
    data.in = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [out]
    data.out = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.in.length;
    length += object.out.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mastering_ros_demo_pkg/demo_srvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6c2950507572d8c02cc39d10ef9010b8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string in
    string out
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new demo_srvRequest(null);
    if (msg.in !== undefined) {
      resolved.in = msg.in;
    }
    else {
      resolved.in = ''
    }

    if (msg.out !== undefined) {
      resolved.out = msg.out;
    }
    else {
      resolved.out = ''
    }

    return resolved;
    }
};

class demo_srvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type demo_srvResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type demo_srvResponse
    let len;
    let data = new demo_srvResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mastering_ros_demo_pkg/demo_srvResponse';
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
    const resolved = new demo_srvResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: demo_srvRequest,
  Response: demo_srvResponse,
  md5sum() { return '6c2950507572d8c02cc39d10ef9010b8'; },
  datatype() { return 'mastering_ros_demo_pkg/demo_srv'; }
};
