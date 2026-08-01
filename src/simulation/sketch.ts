import p5 from "p5";
import type { ParametersState } from "../types/parameters";

export function createSketch(parent: HTMLElement, parameters: ParametersState) {
    return (p: p5) => {
        p.setup = () => {
            p.createCanvas(
                parameters.simulation.canvasWidth,
                parameters.simulation.canvasHeight
            ).parent(parent);
            p.frameRate(24);
        };

        p.draw = () => {
            p.background(100);

            p.fill(255);
            p.noStroke();

            p.fill(255);
            p.textSize(18);
            p.text(`FPS: ${Math.round(p.frameRate())}`, 20, 30);
        };

        p.windowResized = () => {};
    };
}