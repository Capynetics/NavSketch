import type p5 from "p5";
import type { ParametersState } from "../types/parameters";
import type { RoadmapEdge, RoadmapPoint, RoadmapSearch, RrtNode, Simulation, WavefrontGrid } from "./simulation";


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

  draw(p: p5) {
    
    //p.background(220);

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
        const robotX = this.current_state.robot.currentPose.x * 100;
        const robotY = this.current_state.robot.currentPose.y * 100;
        const heading = this.current_state.robot.currentPose.theta;
        const fovRad = (lidar.fieldOfView * Math.PI) / 180;
        const startAngle = heading - fovRad / 2;
        const isFullCircle = Math.abs(fovRad) >= 2 * Math.PI;
        const step = beamCount > 1 ? fovRad / (isFullCircle ? beamCount : beamCount - 1) : 0;

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
      this.current_state.robot.currentPose.x * 100,
      this.current_state.robot.currentPose.y * 100,
      this.current_state.robot.radius * 200,
      this.current_state.robot.radius * 200
    );

    // Draw robot heading line from center to edge.
    const robotX = this.current_state.robot.currentPose.x * 100;
    const robotY = this.current_state.robot.currentPose.y * 100;
    const robotTheta = this.current_state.robot.currentPose.theta;
    const headingLength = this.current_state.robot.radius * 100;
    if (Number.isFinite(robotX) && Number.isFinite(robotY) && Number.isFinite(robotTheta)) {
      p.stroke(0, 0, 180);
      p.strokeWeight(2);
      p.line(
        robotX,
        robotY,
        robotX + Math.cos(robotTheta) * headingLength,
        robotY + Math.sin(robotTheta) * headingLength
      );
    }
    p.noStroke();

    // Draw goal
    p.fill(255, 0, 0);
    p.ellipse(
      this.current_state.goal.x * 100,
      this.current_state.goal.y * 100,
      20,
      20
    );

    // Mark the closest point to goal reached while following a wall
    if (this.simulation.is_bug1_following_wall) {
      p.fill(0, 200, 0);
      p.ellipse(
        this.simulation.min_distance_to_goal_point.x * 100,
        this.simulation.min_distance_to_goal_point.y * 100,
        16,
        16
      );
    }

    // Draw the Bug 2 m-line from the saved start point to the goal.
    if (this.current_state.planner.algorithm === "bug2" && this.simulation.is_bug2_started) {
      p.stroke(0, 170, 0);
      p.strokeWeight(2);
      p.line(
        this.simulation.m_line_x * 100,
        this.simulation.m_line_y * 100,
        this.current_state.goal.x * 100,
        this.current_state.goal.y * 100
      );
      p.noStroke();
    }

    this.drawPlannerOverlay(p);
  }

  private drawPlannerOverlay(p: p5) {
    const { plannerRenderData } = this.simulation;
    switch (this.current_state.planner.algorithm) {
      case "tangentBug":
        this.drawDiscontinuities(p);
        this.drawDiscontinuityLines(p);
        break;
      case "potentialField":
        if (plannerRenderData.potentialFieldForces) {
          const forces = plannerRenderData.potentialFieldForces;
          this.drawPotentialFieldVectors(p, forces.attractiveX, forces.attractiveY, forces.repulsiveX, forces.repulsiveY);
        }
        break;
      case "PRM":
      case "PRM_GRID":
        this.drawRoadMap(p, plannerRenderData.roadmap.points, plannerRenderData.roadmap.edges, plannerRenderData.prmPath, plannerRenderData.prmSearch);
        break;
      case "RRT":
        this.drawRrt(p, plannerRenderData.rrtNodes, plannerRenderData.rrtPath, plannerRenderData.rrtWaypointIndex);
        break;
      case "wavefront":
        this.drawWavefront(p, plannerRenderData.wavefrontGrid, plannerRenderData.wavefrontPath, plannerRenderData.wavefrontWaypointIndex);
        break;
    }
  }

  private drawDiscontinuities(p: p5) {
    const ranges = this.simulation.sensor_ranges;
    const discontinuities = this.simulation.get_discontinuities(ranges);
    const fieldOfView = (this.current_state.lidar.fieldOfView * Math.PI) / 180;
    const startAngle = this.current_state.robot.currentPose.theta - fieldOfView / 2;
    const step = ranges.length > 1 ? fieldOfView / (ranges.length - 1) : 0;
    p.stroke(0, 255, 0);
    p.strokeWeight(10);
    for (const index of discontinuities) {
      const range = Math.min(ranges[index], ranges[index + 1]);
      const angle = startAngle + index * step;
      p.point((this.current_state.robot.currentPose.x + range * Math.cos(angle)) * 100, (this.current_state.robot.currentPose.y + range * Math.sin(angle)) * 100);
    }
  }

  private drawDiscontinuityLines(p: p5) {
    const ranges = this.simulation.sensor_ranges;
    const fieldOfView = (this.current_state.lidar.fieldOfView * Math.PI) / 180;
    const startAngle = this.current_state.robot.currentPose.theta - fieldOfView / 2;
    const step = ranges.length > 1 ? fieldOfView / (ranges.length - 1) : 0;
    p.stroke(255, 165, 0);
    p.strokeWeight(6);
    for (const line of this.simulation.get_discontinuity_lines()) {
      const startRange = Math.min(ranges[line.start], ranges[line.start + 1]);
      const endRange = Math.min(ranges[line.end], ranges[line.end + 1]);
      const startBeamAngle = startAngle + line.start * step;
      const endBeamAngle = startAngle + line.end * step;
      p.line(
        (this.current_state.robot.currentPose.x + startRange * Math.cos(startBeamAngle)) * 100,
        (this.current_state.robot.currentPose.y + startRange * Math.sin(startBeamAngle)) * 100,
        (this.current_state.robot.currentPose.x + endRange * Math.cos(endBeamAngle)) * 100,
        (this.current_state.robot.currentPose.y + endRange * Math.sin(endBeamAngle)) * 100
      );
    }
  }

  private drawPotentialFieldVectors(p: p5, attractiveX: number, attractiveY: number, repulsiveX: number, repulsiveY: number) {
    this.drawForceVector(p, attractiveX, attractiveY, 0, 0, 255);
    this.drawForceVector(p, repulsiveX, repulsiveY, 255, 0, 255);
  }

  private drawForceVector(p: p5, x: number, y: number, r: number, g: number, b: number, scale = 0.5) {
    const origin = this.current_state.robot.currentPose;
    p.stroke(r, g, b);
    p.strokeWeight(4);
    p.line(origin.x * 100, origin.y * 100, (origin.x + x * scale) * 100, (origin.y + y * scale) * 100);
  }

  private drawRoadMap(p: p5, points: RoadmapPoint[], edges: RoadmapEdge[], path: RoadmapPoint[], search: RoadmapSearch | null) {
    if (!this.current_state.visualization.showPlannerGraph) return;
    p.stroke(80, 80, 80);
    p.strokeWeight(1);
    for (const edge of edges) p.line(points[edge.start].x * 100, points[edge.start].y * 100, points[edge.end].x * 100, points[edge.end].y * 100);
    if (search) {
      p.stroke(230, 140, 0); p.strokeWeight(6);
      for (const index of search.closedNodes) p.point(search.points[index].x * 100, search.points[index].y * 100);
      p.stroke(0, 150, 255); p.strokeWeight(7);
      for (const index of search.openNodes) p.point(search.points[index].x * 100, search.points[index].y * 100);
      if (search.currentIndex !== null) { p.stroke(255); p.strokeWeight(10); p.point(search.points[search.currentIndex].x * 100, search.points[search.currentIndex].y * 100); }
    }
    this.drawPath(p, path);
    p.stroke(80, 80, 80); p.strokeWeight(5);
    for (const point of points) p.point(point.x * 100, point.y * 100);
    p.noStroke();
  }

  private drawRrt(p: p5, nodes: RrtNode[], path: RoadmapPoint[], waypointIndex: number) {
    if (!this.current_state.visualization.showPlannerGraph) return;
    p.stroke(70, 120, 180); p.strokeWeight(1);
    for (const node of nodes) if (node.parent !== null) { const parent = nodes[node.parent]; p.line(parent.x * 100, parent.y * 100, node.x * 100, node.y * 100); }
    this.drawPath(p, path.slice(waypointIndex));
    p.noStroke();
  }

  private drawWavefront(p: p5, grid: WavefrontGrid | null, path: RoadmapPoint[], waypointIndex: number) {
    if (!grid || !this.current_state.visualization.showPlannerGraph) return;
    p.noStroke(); p.fill(0, 150, 255, 45); p.textAlign(p.CENTER, p.CENTER); p.textSize(12);
    for (let index = 0; index < grid.distances.length; index += 1) {
      if (grid.distances[index] < 0) continue;
      const column = index % grid.columns; const row = Math.floor(index / grid.columns);
      p.rect(column * grid.cellSize * 100, row * grid.cellSize * 100, grid.cellSize * 100, grid.cellSize * 100);
      p.fill(0); p.text(grid.distances[index], (column + 0.5) * grid.cellSize * 100, (row + 0.5) * grid.cellSize * 100); p.fill(0, 150, 255, 45);
    }
    this.drawPath(p, path.slice(waypointIndex));
    p.noStroke();
  }

  private drawPath(p: p5, path: RoadmapPoint[]) {
    p.stroke(0, 180, 0); p.strokeWeight(3);
    for (let index = 0; index < path.length - 1; index += 1) p.line(path[index].x * 100, path[index].y * 100, path[index + 1].x * 100, path[index + 1].y * 100);
  }
}
