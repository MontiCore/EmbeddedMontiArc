/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";

import * as execa from "execa";

export const DockerMachineService = Symbol("DockerMachineService");
/**
 * An interface to be implemented by classes interacting with the Docker machine by sending commands to it.
 */
export interface DockerMachineService {
    /**
     * Executes the given command on the **default** Docker machine.
     * @param command The command to be executed on the **default** Docker machine (without docker-machine prefix).
     */
    command(command: string): Promise<string>;
}

@injectable()
export class DockerMachineServiceImpl implements DockerMachineService {
    public async command(command: string): Promise<string> {
        const args = command.split(' ');
        const result = await execa("docker-machine", args);

        return result.stdout;
    }
}
