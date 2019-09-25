/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ExecaReturns } from "execa";

import * as execabase from "execa";

export async function execa(command: string, args: string[]): Promise<ExecaReturns> {
    const childProcess = execabase(command, args);

    if (childProcess) {
        if (childProcess.stdout) childProcess.stdout.pipe(process.stdout);
        if (childProcess.stderr) childProcess.stderr.pipe(process.stderr);
    }

    return childProcess;
}
