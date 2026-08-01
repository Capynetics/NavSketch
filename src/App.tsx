
import { useState } from "react";
import Sidebar from './components/sidebar';
import { SimulationCanvas } from "./components/SimulationCanvas";
import type { ParametersState } from "./types/parameters";
import maze1Scenario from "./assets/maze1.json";

function App() {
    const [parameters, setParameters] = useState<ParametersState>({
    simulation: {
        running: false,
        timestep: 0.016,
         canvasWidth: 1280,
         canvasHeight: 720,
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
        x: 12,
        y: 7,
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

    obstacles: maze1Scenario.obstacles,

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
        <div
            style={{
                position: "relative",
                minHeight: "100vh",
                width: "100%",
                overflow: "hidden",
            }}
        >
            <div
                style={{
                    position: "absolute",
                    inset: 0,
                    display: "flex",
                    justifyContent: "center",
                    alignItems: "center",
                }}
            >
                <SimulationCanvas parameters={parameters} />
            </div>
            <Sidebar
                parameters={parameters}
                setParameters={setParameters}
            />
        </div>
  );
}

export default App;