import type { ParametersState } from "../types/parameters";
import type { Simulation } from "./simulation";


export class Render {
  private simulation: Simulation;

  get current_state(): ParametersState {
    return this.simulation.current_state;
  }

  constructor(simulation: Simulation) {
    this.simulation = simulation;
  }

  updateState(nextState: ParametersState) {
    this.simulation.updateState(nextState);
  }

  draw(p: any) {
    
    p.background(220);

    // Draw obstacles
    p.fill(0, 0, 0);
    p.noStroke();
    for (const obstacle of this.current_state.obstacles) {
      const cx = obstacle.pose.x * 100;
      const cy = obstacle.pose.y * 100;

      if (obstacle.type === "rectangle") {
        const width = (obstacle.geometry.width ?? 0) * 100;
        const height = (obstacle.geometry.height ?? 0) * 100;

        p.push();
        p.translate(cx, cy);
        p.rotate(obstacle.pose.theta ?? 0);
        p.rectMode(p.CENTER);
        p.rect(0, 0, width, height);
        p.pop();
      } else if (obstacle.type === "circle") {
        const diameter = (obstacle.geometry.radius ?? 0) * 200;
        p.ellipse(cx, cy, diameter, diameter);
      }
    }

    if (this.current_state.visualization.showLidar) {
      const ranges = this.simulation.sensor_ranges;
      const beamCount = ranges.length;

      if (beamCount > 0) {
        const lidar = this.current_state.lidar;
        const robotX = this.current_state.robot.initialPose.x * 100;
        const robotY = this.current_state.robot.initialPose.y * 100;
        const heading = this.current_state.robot.initialPose.theta;
        const fovRad = (lidar.fieldOfView * Math.PI) / 180;
        const startAngle = heading - fovRad / 2;
        const step = beamCount > 1 ? fovRad / (beamCount - 1) : 0;

        p.stroke(255, 0, 0, 140);
        p.strokeWeight(1);
        for (let i = 0; i < beamCount; i += 1) {
          const angle = startAngle + i * step;
          const distance = ranges[i] * 100;
          p.line(
            robotX,
            robotY,
            robotX + Math.cos(angle) * distance,
            robotY + Math.sin(angle) * distance
          );
        }
        p.noStroke();
      }
    }

    // Draw robot
    p.fill(0, 0, 255);
    p.ellipse(
      this.current_state.robot.initialPose.x * 100,
      this.current_state.robot.initialPose.y * 100,
      this.current_state.robot.radius * 200,
      this.current_state.robot.radius * 200
    );

    // Draw robot heading line from center to edge.
    const robotX = this.current_state.robot.initialPose.x * 100;
    const robotY = this.current_state.robot.initialPose.y * 100;
    const robotTheta = this.current_state.robot.initialPose.theta;
    const headingLength = this.current_state.robot.radius * 100;
    p.stroke(0, 0, 180);
    p.strokeWeight(2);
    p.line(
      robotX,
      robotY,
      robotX + Math.cos(robotTheta) * headingLength,
      robotY + Math.sin(robotTheta) * headingLength
    );
    p.noStroke();

    // Draw goal
    p.fill(255, 0, 0);
    p.ellipse(
      this.current_state.goal.x * 100,
      this.current_state.goal.y * 100,
      20,
      20
    );
   
  } 
}
