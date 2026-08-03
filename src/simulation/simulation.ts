import type { ParametersState } from "../types/parameters";
import { Lidar } from "./sensors/lidar";


export class Simulation {
  current_state: ParametersState;
  sensor_ranges: number[];
  ux: number;
  uy: number;
  private bug0FollowingWall: boolean;
  private bug0WallSide: -1 | 1;
  private lidar: Lidar;

  constructor(current_state: ParametersState) {
    this.current_state = current_state;
    this.sensor_ranges = [];
    this.ux = 0;
    this.uy = 0;
    this.bug0FollowingWall = false;
    this.bug0WallSide = 1;
    this.lidar = new Lidar();
  }

  updateState(nextState: ParametersState) {
    this.current_state = nextState;
  }

  sensor_read() {
    this.sensor_ranges = this.lidar.senseEnvironment(this.current_state);

    return this.sensor_ranges;
  }

  calculate_next_step() {
    switch (this.current_state.planner.algorithm) {
      case "bug0":
        //ToDo
        break;
      case "astar":
        this.bug0FollowingWall = false;
        this.ux = 0;
        this.uy = 0;
        break;
      case "dijkstra":
        this.bug0FollowingWall = false;
        this.ux = 0;
        this.uy = 0;
        break;
      default:
        this.bug0FollowingWall = false;
        this.ux = 0;
        this.uy = 0;
        break;
    }
  }

  move_to_next_step() {
    const dx = this.current_state.goal.x - this.current_state.robot.initialPose.x;
    const dy = this.current_state.goal.y - this.current_state.robot.initialPose.y;
    const distance = Math.sqrt(dx * dx + dy * dy);
    const speed = 2; // m/s

    const dt = this.current_state.simulation.timestep;
    if (distance < speed * dt) {
      this.current_state.robot.initialPose.x = this.current_state.goal.x;
      this.current_state.robot.initialPose.y = this.current_state.goal.y;
      this.current_state.simulation.running = false;
    }else{
      this.current_state.robot.initialPose.x += this.ux * speed * dt;
      this.current_state.robot.initialPose.y += this.uy * speed * dt;
    }
  }
};
