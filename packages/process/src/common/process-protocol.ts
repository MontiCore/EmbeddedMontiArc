/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

export const PROCESS_PATH: string = "/services/emastudio-process";
export const PROCESSES_PATH: string = "/services/emastudio-processes";

export enum ProcessType {
    "Raw", "Terminal"
}

export interface ProcessOptions {
    readonly command: string,
    args?: string[],
    options?: object
}

export interface IProcessExitEvent {
    readonly code: number,
    readonly signal?: string
}

export interface ProcessDescription {
    type: ProcessType;
    options: ProcessOptions;
}

export const ProcessServer = Symbol("ProcessServer");

export interface ProcessServer {
    spawn(options: ProcessDescription): Promise<number | undefined>;
}
