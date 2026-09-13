
import { useState } from "react";
import Sidebar from './components/sidebar';
import { SimulationCanvas } from "./components/SimulationCanvas";
import type { ParametersState } from "./types/parameters";
import bubblesScenario from './assets/bubbles.json'; 

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
        currentPose: {
            x: 1.5,
            y: 4,
            theta: 0,
        },
    },

    goal: {
        x: 12,
        y: 2.5,
    },

    planner: {
        algorithm: "bug1",
    },

    lidar: {
        enabled: true,
        range: 1,
        resolution: 90,
        fieldOfView: 360,
    },

    environment: {
        scenario: "bubbles",
    },

    obstacles: bubblesScenario.obstacles,

    visualization: {
        showGrid: true,
        showRobot: true,
        showGoal: true,
        showObstacles: true,
        showTrajectory: true,
        showLidar: true,
        showRobotHeading: true,
        showCollisionRadius: false,
                showPlannerGraph: true,
    },

    statistics: {
        showSimulationTime: true,
        showDistanceTravelled: true,
    },
  });

  return (
        <div className="app-shell">
            <header className="workspace-header">
                <div className="brand-lockup">
                    <div className="brand-mark"><img src={`${import.meta.env.BASE_URL}logo.svg`} alt="NavSketch logo" /></div>
                    <div>
                        <div className="brand-name">NavSketch</div>
                        <div className="brand-context">Motion planning laboratory</div>
                    </div>
                </div>
                <div className="workspace-status">
                    <span className={`status-dot ${parameters.simulation.running ? 'is-running' : ''}`} />
                    {parameters.simulation.running ? 'Simulation running' : 'Ready to simulate'}
                </div>
            </header>
            <main className="simulation-workspace">
                <div className="canvas-panel">
                    <div className="canvas-panel-header">
                        <div>
                            <div className="eyebrow">Workspace</div>
                            <h1>{parameters.environment.scenario.replaceAll('_', ' ')}</h1>
                        </div>
                        <div className="canvas-legend" aria-label="Simulation legend">
                            <span><i className="legend-swatch robot-swatch" /> Robot</span>
                            <span><i className="legend-swatch goal-swatch" /> Goal</span>
                            <span><i className="legend-swatch lidar-swatch" /> LiDAR</span>
                        </div>
                    </div>
                    <div className="canvas-viewport">
                        <SimulationCanvas parameters={parameters} setParameters={setParameters} />
                    </div>
                    <footer className="canvas-panel-footer">
                        <span><i className="bi bi-cursor-fill" /> Drag robot and goal to reposition</span>
                        <span>{parameters.planner.algorithm} planner</span>
                    </footer>
                </div>
            </main>
            <Sidebar
                parameters={parameters}
                setParameters={setParameters}
            />
        </div>
  );
}

export default App;