export type ScenarioObstacle = {
  id: string;
  type: string;
  pose: {
    x: number;
    y: number;
    theta: number;
  };
  geometry: {
    width?: number;
    height?: number;
    radius?: number;
  };
};

export type ParametersState = {
  simulation: {
    running: boolean;
    timestep: number;
    canvasWidth: number;
    canvasHeight: number;
  };
  robot: {
    model: string;
    radius: number;
    maxLinearVelocity: number;
    maxAngularVelocity: number;
    initialPose: {
      x: number;
      y: number;
      theta: number;
    };
  };
  goal: {
    x: number;
    y: number;
  };
  planner: {
    algorithm: string;
  };
  lidar: {
    enabled: boolean;
    range: number;
    resolution: number;
    fieldOfView: number;
  };
  environment: {
    scenario: string;
  };
  obstacles: ScenarioObstacle[];
  visualization: {
    showGrid: boolean;
    showRobot: boolean;
    showGoal: boolean;
    showObstacles: boolean;
    showTrajectory: boolean;
    showLidar: boolean;
    showRobotHeading: boolean;
    showCollisionRadius: boolean;
    showPlannerGraph: boolean;
  };
  statistics: {
    showSimulationTime: boolean;
    showDistanceTravelled: boolean;
  };
};