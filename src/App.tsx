
import { useState } from "react";
import Sidebar from './components/sidebar';

function App() {
  const [parameters, setParameters] = useState({
    simulation: {
        running: false,
        timestep: 0.016,
    },

    robot: {
        model: "holonomic",      // differential | holonomic | ackermann
        radius: 0.25,
        maxLinearVelocity: 1.0,
        maxAngularVelocity: 2.0,
        initialPose: {
            x: 1,
            y: 1,
            theta: 0,
        },
    },

    goal: {
        x: 8,
        y: 8,
    },

    planner: {
        algorithm: "bug0",
    },

    lidar: {
        enabled: true,
        range: 7,
        resolution: 360,
        fieldOfView: 360,
    },

    environment: {
        scenario: "maze1",
    },

    visualization: {
        showGrid: true,
        showRobot: true,
        showGoal: true,
        showObstacles: true,
        showTrajectory: true,
        showLidar: true,
        showRobotHeading: true,
        showCollisionRadius: false,
        showPlannerGraph: false,
    },

    statistics: {
        showSimulationTime: true,
        showDistanceTravelled: true,
    },
  });

  return (
    <Sidebar
      parameters={parameters}
      setParameters={setParameters}
     />
  );
}

export default App;