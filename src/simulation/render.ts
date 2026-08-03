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
