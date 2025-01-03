---
tags:
    - geo
    - spatial
    - cesium
    - ros2
    - websocket
---

# Cesium ros2 bridge using websocket and rosbridge_server

```bash title="terminal1"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## publish string message

```bash
const rosbridgeWebSocket = new WebSocket("ws://localhost:9090");
```

```js title="advertise Topic"
const advertiseTopic = (topic, type) => {
    const advertiseMessage = {
        op: "advertise",
        topic: topic,
        type: type, // Message type (e.g., std_msgs/String)
    };

    if (rosbridgeWebSocket.readyState === WebSocket.OPEN) {
        rosbridgeWebSocket.send(JSON.stringify(advertiseMessage));
        console.log(`Advertised topic: ${topic} with type: ${type}`);
    } else {
        console.error("WebSocket is not open. Cannot advertise topic.");
    }
};
```

```js title="publish Topic"
const publishMessage = (topic, message) => {
    const rosMessage = {
        op: "publish",
        topic: topic,
        msg: {
            data: message, // The string message
        }
    };

    // Send the message as a JSON string over the WebSocket
    rosbridgeWebSocket.send(JSON.stringify(rosMessage));
};
```

```js title=""
rosbridgeWebSocket.onopen = () => {
    console.log("Connected to rosbridge WebSocket server.");

    // Start publishing a message every second
    const topic = "/example_topic";
    const messageType = "std_msgs/String"; // Specify the ROS 2 message type
    advertiseTopic(topic, messageType);

    let counter = 0;

    setInterval(() => {
        const message = `Hello, ROS 2! Message number: ${counter}`;
        publishMessage(topic, message);
        counter++; // Increment counter for each message
    }, 1000); // 1000 ms = 1 second
};

rosbridgeWebSocket.onerror = (error) => {
    console.error("WebSocket Error:", error);
};

rosbridgeWebSocket.onclose = () => {
    console.log("WebSocket connection closed.");
};
```


```bash
ros2 topic echo /example_topic
```

## receive string message

- launch webbridge
- pub message
- reload cesium web page

!!! note "order of execution"
    The bridge need to infer the message
    run the pub method before open the websocket from cesium
     
```bash
ros2 topic pub /example_topic std_msgs/String "{ data: 'hello world' }"
```

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```