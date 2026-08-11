import { useEffect, useRef, type Dispatch, type SetStateAction } from "react";
import p5 from "p5";
import { createSketch } from "../simulation/sketch";
import type { Simulation } from "../simulation/simulation";
import type { ParametersState } from "../types/parameters";

type SimulationCanvasProps = {
    parameters: ParametersState;
    setParameters: Dispatch<SetStateAction<ParametersState>>;
};

export function SimulationCanvas({ parameters, setParameters }: SimulationCanvasProps) {
    const containerRef = useRef<HTMLDivElement>(null);
    const instanceRef = useRef<p5 | null>(null);
    const simulationRef = useRef<Simulation | null>(null);

    useEffect(() => {
        if (!containerRef.current) return;

        const onRobotDrag = (x: number, y: number) => {
            setParameters((value) => ({
                ...value,
                robot: {
                    ...value.robot,
                    currentPose: { ...value.robot.currentPose, x, y },
                },
            }));
        };

        const onGoalDrag = (x: number, y: number) => {
            setParameters((value) => ({
                ...value,
                goal: { ...value.goal, x, y },
            }));
        };

        const sketch = createSketch(containerRef.current, parameters, simulationRef, {
            onRobotDrag,
            onGoalDrag,
        });
        const instance = new p5(sketch);
        instanceRef.current = instance;

        return () => {
            instance.remove();
            instanceRef.current = null;
            simulationRef.current = null;
        };
    }, []);

    useEffect(() => {
        if (!simulationRef.current) return;

        simulationRef.current.updateState(parameters);
    }, [parameters]);

    useEffect(() => {
        if (!instanceRef.current) return;

        instanceRef.current.resizeCanvas(
            parameters.simulation.canvasWidth,
            parameters.simulation.canvasHeight
        );
    }, [parameters.simulation.canvasWidth, parameters.simulation.canvasHeight]);

    return (
        <div
            ref={containerRef}
            style={{
                width: `${parameters.simulation.canvasWidth}px`,
                height: `${parameters.simulation.canvasHeight}px`,
            }}
        />
    );
}