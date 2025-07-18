/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CommonFrontendConfigurationRunner } from "@embeddedmontiarc/sol-runtime-configurations/lib/browser";
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { injectable } from "inversify";
import { TestConfigurationOptions } from "../common";
import { CancellationToken } from "@theia/core/lib/common";

@injectable()
export class TestConfigurationRunner extends CommonFrontendConfigurationRunner<TestConfigurationOptions> {
    public constructor() {
        super("id");
    }

    public async run(uuid: string, taskName: string, options: TestConfigurationOptions, context: OptionsContext, token: CancellationToken): Promise<void> {
        switch (taskName) {
            case "logName": return this.runLogName(uuid, options, context, token);
        }
    }

    protected async runLogName(uuid: string, options: TestConfigurationOptions, context: OptionsContext, token: CancellationToken): Promise<void> {
        console.log(options.name);
    }
}
