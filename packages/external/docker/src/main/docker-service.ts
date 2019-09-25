/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";
import { ExecaChildProcess } from "execa";

import * as execa from "execa";

import vector from "string-argv";

export const DockerService = Symbol("DockerService");
/**
 * An interface to be implemented by classes which implement the necessary functionality to execute docker commands.
 */
export interface DockerService {
    /**
     * Executes the given docker command.
     * @param command The docker command to be executed (without docker prefix).
     * @return The standard output of the executed docker command.
     */
    command(command: string): Promise<string>;

    /**
     * Executes the given docker command.
     * @param command The docker command to be executed.
     * @return The process spawned from the given command.
     */
    commandAsProcess(command: string): ExecaChildProcess;
}

@injectable()
export class DockerServiceImpl implements DockerService {
    public async command(command: string): Promise<string> {
        const args = vector(command);
        const result = await execa("docker", args);

        return result.stdout;
    }

    public commandAsProcess(command: string): ExecaChildProcess {
        const args = vector(command);

        return execa("docker", args);
    }
}
