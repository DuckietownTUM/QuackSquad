import React, { useState, useEffect, useCallback } from "react";
import roslib from "roslib";
import { Joystick } from "react-joystick-component";
import CourseInformation from "./components/CourseInformation";
// import { Button } from "@/components/ui/button";

const ROSBridgeURL = "ws://duckie.local:9090"; // Change this if needed
const topicName = "/duckie/joy";

const App = () => {
  const [ros, setRos] = useState(null);
  const [publisher, setPublisher] = useState(null);
  const [eStopTopic, setEStopTopic] = useState(null);
  const [idleModeTopic, setIdleModeTopic] = useState(null);
  const [emergencyActive, setEmergencyActive] = useState(false);
  const [isIdle, setIsIdle] = useState(true);
  const [startPoint, setStartPoint] = useState("");
  const [destinationPoint, setDestinationPoint] = useState("");
  const [computePathSrv, setComputePathSrv] = useState(null)
  const [dijkstraPath, setDijkstraPath] = useState(null)

  useEffect(() => {
    const ros = new roslib.Ros({ url: ROSBridgeURL });

    ros.on("connection", () => {
      console.log("Connected to ROS")
  
      document.getElementById("status").textContent = "Connected";
      document.getElementById("status").classList.remove("text-red-600");
      document.getElementById("status").classList.add("text-green-600");
  
      let pub = new roslib.Topic({
        ros: ros,
        name: topicName,
        messageType: "sensor_msgs/Joy",
      });
  
      let eStopTopic = new roslib.Topic({
        ros: ros,
        name: "/duckie/wheels_driver_node/emergency_stop",
        messageType: "duckietown_msgs/BoolStamped"
      });
  
      let idleModeTopic = new roslib.Topic({
        ros: ros,
        name: "/duckie/joy_mapper_node/idle_mode",
        messageType: "duckietown_msgs/BoolStamped"
      });
  
      // Connect to the service
      let computePathSrv = new roslib.Service({
        ros: ros,
        name: '/duckie/dijkstra_turns_node/compute_path',
        serviceType: 'dijkstra/SetRoute', // Change the type if the service has a specific one
      });
  
      eStopTopic.subscribe(handleEStop);
      idleModeTopic.subscribe(handleIdleMode);
  
      setRos(ros);
      setPublisher(pub);
      setEStopTopic(eStopTopic)
      setIdleModeTopic(idleModeTopic)
      setComputePathSrv(computePathSrv)
    })
  
    ros.on("error", (error) => {
      console.error("ROS error: ", error)
  
      document.getElementById("status").textContent = "Error Connecting";
      document.getElementById("status").classList.remove("text-green-600");
      document.getElementById("status").classList.add("text-red-600");
    })
  
    ros.on("close", () => {
      console.log("Disconnected from ROS")
  
      document.getElementById("status").textContent = "Disconnected";
      document.getElementById("status").classList.remove("text-green-600");
      document.getElementById("status").classList.add("text-red-600");
    })

    return () => {
      ros.close();
    };
  }, []);


  const sendJoyMessage = (axes, buttons) => {
    if (publisher) {
      const message = new roslib.Message({
        axes,
        buttons,
      });
      publisher.publish(message);
    }
  };

  const handleMove = (event) => {
    const x = event.y < 0 ? event.x : -event.x || 0;
    const y = event.y || 0;
    sendJoyMessage([0, y, 0, x], Array(15).fill(0));
  };

  
  const handleEStop = (msg) => {
    setEmergencyActive(msg.data);
    let startBtn = document.getElementById("startBtn")
    if (msg.data) {
      setIsIdle(true); // Reset to idle mode
      startBtn.style.display = "block";
      startBtn.classList.add("opacity-50", "cursor-not-allowed");
    }
    else {
      startBtn.classList.remove("opacity-50", "cursor-not-allowed");
    }
  };
  
  const handleIdleMode = (msg) => {
    setIsIdle(msg.data);
    let startBtn = document.getElementById("startBtn");

    if (!msg.data) {
      startBtn.style.display = "none";
    }
    else {
      startBtn.style.display = "block";
    }
  };

  const handleStop = () => {
    sendJoyMessage([0, 0, 0, 0], Array(15).fill(0));
  };


  useEffect(() => {
    const pressedKeys = new Set();

    const updateAxes = () => {
      let axes = [0, 0, 0, 0];

      if (pressedKeys.has("ArrowUp")) axes[1] = 1;  // Forward
      if (pressedKeys.has("ArrowDown")) axes[1] = -1; // Backward
      if (pressedKeys.has("ArrowLeft")) axes[3] = 1; // Left
      if (pressedKeys.has("ArrowRight")) axes[3] = -1; // Right

      sendJoyMessage(axes, Array(15).fill(0));
    };

    const handleKeyDown = (event) => {
      if (["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(event.key)) {
        pressedKeys.add(event.key);
        updateAxes();
      }

      // Check for Ctrl + Shift + /
      if (event.ctrlKey && event.shiftKey && event.key === "/") {
        let buttons = Array(15).fill(0);
        buttons[6] = 1;
        sendJoyMessage([0, 0, 0, 0], buttons);
      }
    }

    const handleKeyUp = (event) => {
      pressedKeys.delete(event.key);
      updateAxes();
    };


    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
  });
  
  const startCourse = () => {
    sendJoyMessage([0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]);
  }

  const toggleEmergencyStop = () => {
    sendJoyMessage([0, 0, 0, 0], [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
  };

  const computePath = () => {
    // Regex to match [digit][any character][digit]
    const regex = /^\d\D\d$/;

    // Validate start and destination points
    if (!regex.test(startPoint) || !regex.test(destinationPoint)) {
      alert("Both start and destination points should be in the format [digit][any character][digit] (e.g., 1a1).");
      return;
    }

    let req = new roslib.ServiceRequest({
      start_point: {x: parseFloat(startPoint[0]), y: parseFloat(startPoint[2]), z:0.0},
      dest_point: { x: parseFloat(destinationPoint[0]), y: parseFloat(destinationPoint[2]), z:0.0}
    });

    computePathSrv.callService(req, (res) => {
      console.log(res)
      if (res.success) {
        console.log("Path computed successfully!");
        setDijkstraPath(res.path)
        console.log(dijkstraPath)
      }
      else {
        console.log("Failed to compute path");
      }
    })
  }

  return (
    <div className="flex flex-col items-center p-6 bg-gray-100 min-h-screen">
      <p class="text-lg mb-4">Status: <span id="status" class="font-semibold text-red-600">Disconnected</span></p>
      <div className="flex flex-col items-center bg-white shadow-lg rounded-lg p-8 mb-8 w-full max-w-md text-center">
        <h1 className="text-2xl font-bold mb-4">📍 Duckiebot Path Finder</h1>
        <div className="flex flex-col gap-2 mt-4">
          <input
            type="text"
            placeholder="Enter start point"
            value={startPoint}
            onChange={(e) => setStartPoint(e.target.value)}
            className="w-full px-4 py-2 border rounded mb-2"
          />
          <input
            type="text"
            placeholder="Enter destination point"
            value={destinationPoint}
            onChange={(e) => setDestinationPoint(e.target.value)}
            className="w-full px-4 py-2 border rounded mb-4"
          />
          <button id="computePath" onClick={computePath} disabled={emergencyActive || !startPoint || !destinationPoint} className={`px-12 py-2 rounded w-full bg-yellow-400 ${emergencyActive || !startPoint || !destinationPoint ? "opacity-50 cursor-not-allowed" : "hover:bg-yellow-500 text-white"}`} >
            Continue
          </button>
        </div>
      </div>

      <div className="flex flex-col items-center bg-white shadow-lg rounded-lg p-8 mb-8 w-full max-w-md text-center">
        <h1 className="text-2xl font-bold mb-4">🎮 Duckiebot Joystick</h1>
        <Joystick className="flex flex-col gap-2 mt-4" size={150} baseColor="#ddd" stickColor="#999" move={handleMove} stop={handleStop} />

        <div className="flex flex-col gap-2 mt-4">
          <button id="startBtn" onClick={startCourse} disabled={emergencyActive} className="bg-green-500 text-white px-12 py-2 rounded w-full hover:bg-green-700">
            ▶ Start / Resume
          </button>
          <button id="eStopBtn" onClick={toggleEmergencyStop} className="bg-red-500 text-white px-12 py-2 rounded w-full mt-2 hover:bg-red-700">
            ⛔ Emergency Stop
          </button>
        </div>
      </div>
      <CourseInformation/>
    </div>
  );
};

export default App;
