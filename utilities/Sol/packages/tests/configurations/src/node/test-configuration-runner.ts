/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CommonBackendConfigurationRunner } from "@embeddedmontiarc/sol-runtime-configurations/lib/node";
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { ToolFactory } from "@embeddedmontiarc/sol-runtime-artifact/lib/node/tool-factory";
import { JavaVirtualToolFactory } from "@embeddedmontiarc/sol-tests-artifact/lib/node/java-virtual-tool";
import { CancellationToken } from "@theia/core/lib/common";
import { FileUri } from "@theia/core/lib/node";
import { inject, injectable } from "inversify";
import { TestConfigurationOptions } from "../common";

import * as path from "path";

@injectable()
export class TestConfigurationRunner extends CommonBackendConfigurationRunner<TestConfigurationOptions> {
    @inject(JavaVirtualToolFactory) protected readonly javaFactory: ToolFactory;

    public constructor() {
        super("id");
    }

    public async run(uuid: string, taskName: string, options: TestConfigurationOptions, context: OptionsContext, token: CancellationToken): Promise<void> {
        switch (taskName) {
            case "showPath": return this.runShowPath(uuid, options, context, token);
        }
    }

    protected async runShowPath(uuid: string, options: TestConfigurationOptions, context: OptionsContext, token: CancellationToken): Promise<void> {
        const process = this.javaFactory("-version", uuid);

        await this.waitForTermination(process, token);

        if (context.workspace) {
            const fsPath = FileUri.fsPath(context.workspace);

            return new Promise(resolve => {
                console.log(path.join(fsPath, options.path));
                setTimeout(resolve, 10000);
                token.onCancellationRequested(resolve);
            });
        }
    }
}
