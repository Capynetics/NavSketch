import { useState, type Dispatch, type SetStateAction } from 'react';
import type { ParametersState } from '../types/parameters';
import maze1Scenario from '../assets/maze1.json';

const scenarioObstaclesByName: Record<string, ParametersState['obstacles']> = {
  maze1: maze1Scenario.obstacles,
};

type SidebarProps = {
  parameters: ParametersState;
  setParameters: Dispatch<SetStateAction<ParametersState>>;
};

function Sidebar({ parameters, setParameters }: SidebarProps) {
  const [collapsed, setCollapsed] = useState(false);
  const [robotCollapsed, setRobotCollapsed] = useState(true);
  const [simulationCollapsed, setSimulationCollapsed] = useState(true);

  const updateSimulation = (patch: Partial<ParametersState['simulation']>) => {
    setParameters((value) => ({
      ...value,
      simulation: {
        ...value.simulation,
        ...patch,
      },
    }));
  };

  const updateRobot = (patch: Partial<ParametersState['robot']>) => {
    setParameters((value) => ({
      ...value,
      robot: {
        ...value.robot,
        ...patch,
      },
    }));
  };

  const updateGoal = (patch: Partial<ParametersState['goal']>) => {
    setParameters((value) => ({
      ...value,
      goal: {
        ...value.goal,
        ...patch,
      },
    }));
  };

  const updateCurrentPose = (patch: Partial<ParametersState['robot']['currentPose']>) => {
    setParameters((value) => ({
      ...value,
      robot: {
        ...value.robot,
        currentPose: {
          ...value.robot.currentPose,
          ...patch,
        },
      },
    }));
  };

  const updatePlanner = (patch: Partial<ParametersState['planner']>) => {
    setParameters((value) => ({
      ...value,
      planner: {
        ...value.planner,
        ...patch,
      },
    }));
  };

  const updateLidar = (patch: Partial<ParametersState['lidar']>) => {
    setParameters((value) => ({
      ...value,
      lidar: {
        ...value.lidar,
        ...patch,
      },
    }));
  };

  const updateEnvironment = (patch: Partial<ParametersState['environment']>) => {
    setParameters((value) => ({
      ...value,
      environment: {
        ...value.environment,
        ...patch,
      },
    }));
  };

  const updateVisualization = (patch: Partial<ParametersState['visualization']>) => {
    setParameters((value) => ({
      ...value,
      visualization: {
        ...value.visualization,
        ...patch,
      },
    }));
  };

  const updateStatistics = (patch: Partial<ParametersState['statistics']>) => {
    setParameters((value) => ({
      ...value,
      statistics: {
        ...value.statistics,
        ...patch,
      },
    }));
  };

  return (
    <aside
      className="bg-dark text-white p-3 d-flex flex-column gap-3"
      style={{
        position: 'absolute',
        top: 0,
        left: 0,
        zIndex: 10,
        width: collapsed ? '72px' : '320px',
        transition: 'width 0.2s ease-in-out',
        minHeight: '100vh',
        height: '100vh',
        overflowX: 'hidden',
        overflowY: 'auto',
        flexShrink: 0,
        boxShadow: '0 0 24px rgba(0, 0, 0, 0.35)',
      }}
    >
      <div className={`d-flex align-items-center ${collapsed ? 'justify-content-center' : 'justify-content-between'}`}>
        {!collapsed && <span className="fw-semibold small text-uppercase text-secondary">Controls</span>}
        <button
          type="button"
          className="btn btn-outline-light btn-sm flex-shrink-0"
          onClick={() => setCollapsed((value) => !value)}
          aria-expanded={!collapsed}
          aria-label={collapsed ? 'Expand sidebar' : 'Collapse sidebar'}
        >
          <i className={`bi ${collapsed ? 'bi-chevron-double-right' : 'bi-chevron-double-left'}`} />
        </button>
      </div>

      {!collapsed && (
        <>
          <div className="bg-secondary bg-opacity-10 rounded-3 p-3">
            <button
              type="button"
              className="btn btn-link p-0 text-white text-decoration-none w-100 d-flex align-items-center justify-content-between mb-3"
              onClick={() => setRobotCollapsed((value) => !value)}
            >
              <h6 className="fw-semibold mb-0">Robot</h6>
              <i className={`bi ${robotCollapsed ? 'bi-chevron-down' : 'bi-chevron-up'}`} />
            </button>

            {!robotCollapsed && (
              <>
                <div className="mb-3">
              <label className="form-label small text-secondary">Model</label>
              <select
                className="form-select form-select-sm"
                value={parameters.robot.model}
                onChange={(event) => updateRobot({ model: event.target.value })}
              >
                <option value="differential">Differential</option>
                <option value="holonomic">Holonomic</option>
                <option value="ackermann">Ackermann</option>
              </select>
            </div>

            <div className="mb-3">
              <label className="form-label small text-secondary">Radius</label>
              <input
                type="number"
                className="form-control form-control-sm"
                value={parameters.robot.radius}
                step="0.01"
                onChange={(event) => updateRobot({ radius: Number(event.target.value) })}
              />
            </div>

            <div className="row g-2 mb-3">
              <div className="col-6">
                <label className="form-label small text-secondary">Max Linear</label>
                <input
                  type="number"
                  className="form-control form-control-sm"
                  value={parameters.robot.maxLinearVelocity}
                  step="0.1"
                  onChange={(event) => updateRobot({ maxLinearVelocity: Number(event.target.value) })}
                />
              </div>
              <div className="col-6">
                <label className="form-label small text-secondary">Max Angular</label>
                <input
                  type="number"
                  className="form-control form-control-sm"
                  value={parameters.robot.maxAngularVelocity}
                  step="0.1"
                  onChange={(event) => updateRobot({ maxAngularVelocity: Number(event.target.value) })}
                />
              </div>
            </div>

            <div className="mb-2 small text-secondary">Current Pose</div>
            <div className="row g-2 mb-2">
              <div className="col-4">
                <label className="form-label small text-secondary">X</label>
                <input
                  type="number"
                  className="form-control form-control-sm"
                  value={parameters.robot.currentPose.x}
                  onChange={(event) => updateCurrentPose({ x: Number(event.target.value) })}
                />
              </div>
              <div className="col-4">
                <label className="form-label small text-secondary">Y</label>
                <input
                  type="number"
                  className="form-control form-control-sm"
                  value={parameters.robot.currentPose.y}
                  onChange={(event) => updateCurrentPose({ y: Number(event.target.value) })}
                />
              </div>
              <div className="col-4">
                <label className="form-label small text-secondary">θ</label>
                <input
                  type="number"
                  className="form-control form-control-sm"
                  value={parameters.robot.currentPose.theta}
                  onChange={(event) => updateCurrentPose({ theta: Number(event.target.value) })}
                />
              </div>
            </div>

                <div className="row g-2">
                  <div className="col-6">
                    <label className="form-label small text-secondary">Goal X</label>
                    <input
                      type="number"
                      className="form-control form-control-sm"
                      value={parameters.goal.x}
                      onChange={(event) => updateGoal({ x: Number(event.target.value) })}
                    />
                  </div>
                  <div className="col-6">
                    <label className="form-label small text-secondary">Goal Y</label>
                    <input
                      type="number"
                      className="form-control form-control-sm"
                      value={parameters.goal.y}
                      onChange={(event) => updateGoal({ y: Number(event.target.value) })}
                    />
                  </div>
                </div>
              </>
            )}
          </div>

          <div className="bg-secondary bg-opacity-10 rounded-3 p-3">
            <button
              type="button"
              className="btn btn-link p-0 text-white text-decoration-none w-100 d-flex align-items-center justify-content-between mb-3"
              onClick={() => setSimulationCollapsed((value) => !value)}
            >
              <h6 className="fw-semibold mb-0">Simulation</h6>
              <i className={`bi ${simulationCollapsed ? 'bi-chevron-down' : 'bi-chevron-up'}`} />
            </button>

            {!simulationCollapsed && (
              <>
            <div className="mb-3">
              <label className="form-label small text-secondary">Timestep</label>
              <input
                type="number"
                className="form-control form-control-sm"
                value={parameters.simulation.timestep}
                step="0.001"
                onChange={(event) => updateSimulation({ timestep: Number(event.target.value) })}
              />
            </div>

            <div className="row g-2 mb-3">
              <div className="col-6">
                <label className="form-label small text-secondary">Canvas Width</label>
                <input
                  type="number"
                  className="form-control form-control-sm"
                  value={parameters.simulation.canvasWidth}
                  step="1"
                  onChange={(event) => updateSimulation({ canvasWidth: Number(event.target.value) })}
                />
              </div>
              <div className="col-6">
                <label className="form-label small text-secondary">Canvas Height</label>
                <input
                  type="number"
                  className="form-control form-control-sm"
                  value={parameters.simulation.canvasHeight}
                  step="1"
                  onChange={(event) => updateSimulation({ canvasHeight: Number(event.target.value) })}
                />
              </div>
            </div>

            <div className="mb-3">
              <label className="form-label small text-secondary">Planner</label>
              <select
                className="form-select form-select-sm"
                value={parameters.planner.algorithm}
                onChange={(event) => updatePlanner({ algorithm: event.target.value })}
              >
                <option value="bug0">Bug0</option>
                <option value="astar">A*</option>
                <option value="dijkstra">Dijkstra</option>
              </select>
            </div>

            <div className="form-check form-switch mb-2">
              <input
                className="form-check-input"
                type="checkbox"
                checked={parameters.lidar.enabled}
                onChange={() => updateLidar({ enabled: !parameters.lidar.enabled })}
                id="lidar-enabled"
              />
              <label className="form-check-label small text-secondary" htmlFor="lidar-enabled">
                Enable LiDAR
              </label>
            </div>

            <div className="row g-2 mb-3">
              <div className="col-6">
                <label className="form-label small text-secondary">Range</label>
                <input
                  type="number"
                  className="form-control form-control-sm"
                  value={parameters.lidar.range}
                  onChange={(event) => updateLidar({ range: Number(event.target.value) })}
                />
              </div>
              <div className="col-6">
                <label className="form-label small text-secondary">Resolution</label>
                <input
                  type="number"
                  className="form-control form-control-sm"
                  value={parameters.lidar.resolution}
                  onChange={(event) => updateLidar({ resolution: Number(event.target.value) })}
                />
              </div>
            </div>

            <div className="mb-3">
              <label className="form-label small text-secondary">Environment</label>
              <select
                className="form-select form-select-sm"
                value={parameters.environment.scenario}
                onChange={(event) => {
                  const scenario = event.target.value;
                  updateEnvironment({ scenario });

                  setParameters((value) => ({
                    ...value,
                    obstacles: scenarioObstaclesByName[scenario] ?? [],
                  }));
                }}
              >
                <option value="maze1">Maze 1</option>
                <option value="maze2">Maze 2</option>
                <option value="open">Open</option>
              </select>
            </div>

            <div className="mb-3">
              <div className="small text-secondary mb-2">Visualization</div>
              <div className="row g-2">
                {[
                  ['showGrid', 'Grid'],
                  ['showRobot', 'Robot'],
                  ['showGoal', 'Goal'],
                  ['showObstacles', 'Obstacles'],
                  ['showTrajectory', 'Trajectory'],
                  ['showLidar', 'LiDAR'],
                  ['showRobotHeading', 'Heading'],
                  ['showCollisionRadius', 'Collision Radius'],
                  ['showPlannerGraph', 'Planner Graph'],
                ].map(([key, label]) => (
                  <div className="col-6" key={key}>
                    <div className="form-check form-switch">
                      <input
                        className="form-check-input"
                        type="checkbox"
                        checked={parameters.visualization[key as keyof ParametersState['visualization']] as boolean}
                        onChange={() =>
                          updateVisualization({
                            [key]: !parameters.visualization[key as keyof ParametersState['visualization']],
                          } as Partial<ParametersState['visualization']>)
                        }
                        id={key}
                      />
                      <label className="form-check-label small text-secondary" htmlFor={key}>
                        {label}
                      </label>
                    </div>
                  </div>
                ))}
              </div>
            </div>

                <div className="small text-secondary mb-2">Statistics</div>
                <div className="row g-2">
                  {[
                    ['showSimulationTime', 'Simulation Time'],
                    ['showDistanceTravelled', 'Distance'],
                  ].map(([key, label]) => (
                    <div className="col-6" key={key}>
                      <div className="form-check form-switch">
                        <input
                          className="form-check-input"
                          type="checkbox"
                          checked={parameters.statistics[key as keyof ParametersState['statistics']] as boolean}
                          onChange={() =>
                            updateStatistics({
                              [key]: !parameters.statistics[key as keyof ParametersState['statistics']],
                            } as Partial<ParametersState['statistics']>)
                          }
                          id={key}
                        />
                        <label className="form-check-label small text-secondary" htmlFor={key}>
                          {label}
                        </label>
                      </div>
                    </div>
                  ))}
                </div>
              </>
            )}
          </div>
        </>
      )}

      <div className="mt-auto pt-2 position-sticky bottom-0 bg-dark">
        <button
          type="button"
          className={`btn w-100 ${parameters.simulation.running ? 'btn-danger' : 'btn-success'}`}
          onClick={() => updateSimulation({ running: !parameters.simulation.running })}
          aria-pressed={parameters.simulation.running}
          aria-label={parameters.simulation.running ? 'Pause simulation' : 'Play simulation'}
        >
          <i className={`bi ${parameters.simulation.running ? 'bi-pause-fill' : 'bi-play-fill'} me-2`} />
          {!collapsed && (parameters.simulation.running ? 'Pause Simulation' : 'Play Simulation')}
        </button>
      </div>
    </aside>
  );
}

export default Sidebar;
