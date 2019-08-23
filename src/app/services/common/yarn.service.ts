/* (c) https://github.com/MontiCore/monticore */
import { Injectable } from "@angular/core";

import * as childProcess from "child_process";
import { WorkspaceService } from "@services/common/workspace.service";

@Injectable({ providedIn: "root" })
export class YarnService {
    public constructor(protected readonly workspace: WorkspaceService) {}

    public install(): Promise<void> {
        return new Promise(async resolve => {
            const rootPath = await this.workspace.getRoot();
            const process = childProcess.exec("yarn install", { cwd: rootPath });

            process.on("exit", resolve);
        });
    }
}
