import type { ParametersState } from "../types/parameters";


export class Simulation {
  current_state: ParametersState;

  constructor(current_state: ParametersState) {
    this.current_state = current_state;
  }

  updateState(nextState: ParametersState) {
    this.current_state = nextState;
  }

  calculate_next_step() {
    const dx = this.current_state.goal.x - this.current_state.robot.initialPose.x;
    const dy = this.current_state.goal.y - this.current_state.robot.initialPose.y;
    const distance = Math.sqrt(dx * dx + dy * dy);
    const ux = dx / distance;
    const uy = dy / distance;
    const speed = 2; // m/s

    const dt = this.current_state.simulation.timestep;
    if (distance < speed * dt) {
      this.current_state.robot.initialPose.x = this.current_state.goal.x;
      this.current_state.robot.initialPose.y = this.current_state.goal.y;
      this.current_state.simulation.running = false;
    }else{
      this.current_state.robot.initialPose.x += ux * speed * dt;
      this.current_state.robot.initialPose.y += uy * speed * dt;
    }
  }

  draw(p: any) {
    
    p.background(220);

    // Draw robot
    p.fill(0, 0, 255);
    p.ellipse(
      this.current_state.robot.initialPose.x * 100,
      this.current_state.robot.initialPose.y * 100,
      this.current_state.robot.radius * 200,
      this.current_state.robot.radius * 200
    );

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
