/*
 * (c) https://github.com/MontiCore/monticore
 */
import { BackendApplicationContribution } from "@theia/core/lib/node";
import { injectable } from "inversify";
import { CommonConfigurationRunnerRegistry } from "../../common";

@injectable()
export class BackendConfigurationRunnerRegistryImpl extends CommonConfigurationRunnerRegistry implements BackendApplicationContribution {
    public onStart(): void {
        this.contributions.getContributions().forEach(contribution => contribution.registerRunners(this));
    }
}
