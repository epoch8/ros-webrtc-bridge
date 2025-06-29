# webrtc-bridge

webrtc-bridge is a ROS 2 package which provides functionality of WebRTC
streaming for image topics and transports control messages back to the ROS
system

webrtc-bridge implements two nodes:

* `webrtc_bridge_local_node` - implements webrtc streaming for "local" mode,
  alongside `rosbridge` as a carrier of control messages, intended use case is to
  use `roslibjs`<->`rosbridge` on web-browser side to do communication and use
  `webrtc_bridge_local_node` to perform streaming
* `webrtc_bridge_remote_node` - designed for a remote use case, when browser and
  ROS system are located in different networks. In this case
  `webrtc_bridge_remote_node` uses external service for signaling and carries
  all the data signals using WebRTC technology

# Current state

| Platform | Feature | Status |
| --- | --- | --- |
| x86 | Streaming | ✅ (with software encoding) |
| rpi4 | Streaming | ✅ (with software encoding) |
| jetson orin nano | Streaming | ❔ |
