/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CommonConfigurationCoordinator } from "@embeddedmontiarc/sol-runtime-configurations/lib/node";
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { CancellationToken } from "@theia/core/lib/common";
import { injectable } from "inversify";
import { TestConfigurationOptions } from "../common";

@injectable()
export class TestConfigurationCoordinator extends CommonConfigurationCoordinator<TestConfigurationOptions> {
    public constructor() {
        super("id");
    }

    public async run(uuid: string, options: TestConfigurationOptions, context: OptionsContext, token: CancellationToken): Promise<void> {
        await this.frontend.run(uuid, this.id, "logName", options, context);
        return this.backend.run(uuid, this.id, "showPath", options, context, token);
    }
}
