import type { Vec2 } from "./vec2";

export interface OptimizableVariableData
{
    range_min: number,
    range_max: number,
    per_layer: number,
    tolerance: number,
}

export interface OptimizablePolarVec2Data
{
    direction?: OptimizableVariableData,
    magnitude?: OptimizableVariableData,
}

export interface OptimizableVec2Data
{
    x?: OptimizableVariableData,
    y?: OptimizableVariableData,
}

export async function optimize<T extends { [key: string]: number }>(loss_function: (variables: T) => Promise<number>, variables: { [K in keyof T]: OptimizableVariableData }): Promise<T>
{
    const variable_keys = Object.keys(variables);

    async function find_min(in_current_variables: T, current_variable_keys: (keyof T)[]): Promise<[number, T]>
    {
        const key = current_variable_keys.pop()!;
        const data = variables[key];

        let min = Infinity;
        let min_variables = in_current_variables;
        for (let t = 0; t < data.per_layer; t++)
        {
            const gap = (data.range_max - data.range_min) / (data.per_layer + 1);
            const x = data.range_min + (gap / 2) + t * gap;

            let current_error: number;
            let current_variables = {...in_current_variables, [key]: x};
            if (current_variable_keys.length > 0)
            {
                [current_error, current_variables] = await find_min({...current_variables}, [...current_variable_keys]);
            }
            else
            {
                current_error = await loss_function(current_variables);
            }

            if (!isNaN(current_error) && current_error < min)
            {
                min = current_error;
                min_variables = current_variables;
            }
        }

        return [min, min_variables!];
    }

    return (await find_min(
        variable_keys.reduce((o, k) => {return { [k]: 0, ...o }}, {} as Partial<T>) as T,
        variable_keys
    ))[1];
}

export async function layered_optimize<T extends { [key: string]: number }>(layers: number, loss_function: (variables: T) => Promise<number>, variables: { [K in keyof T]: OptimizableVariableData }): Promise<T>
{
    let current_variable_ranges = {...variables};
    let current_variable_values = await optimize(loss_function, variables);

    for (let layer = 0; layer < layers; layer++)
    {
        console.log(`layer: ${layer}`);

        let in_tolerance = true;
        for (const variable_key in variables)
        {
            const current_range = current_variable_ranges[variable_key];
            const last_gap = (current_range.range_max - current_range.range_min) / current_range.per_layer;

            const variable_in_tolerance = (last_gap < current_range.tolerance);
            in_tolerance = in_tolerance && variable_in_tolerance;
            
            if (variable_in_tolerance)
            {
                current_range.range_min = current_variable_values[variable_key];
                current_range.range_max = current_variable_values[variable_key];
                current_range.per_layer = 1;
            }
            else
            {
                current_range.range_min = current_variable_values[variable_key] - last_gap / 2;
                current_range.range_max = current_variable_values[variable_key] + last_gap / 2;

                current_range.per_layer = Math.min(current_range.per_layer, Math.ceil((current_range.range_max - current_range.range_min) / current_range.tolerance));
            }
        }

        if (in_tolerance)
        {
            console.log("made tolerance");
            break;
        }

        current_variable_values = await optimize(loss_function, current_variable_ranges);
    }

    return current_variable_values!;
}