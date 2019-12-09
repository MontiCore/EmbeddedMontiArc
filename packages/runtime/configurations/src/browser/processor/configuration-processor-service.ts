/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ValidatorService } from "@embeddedmontiarc/sol-runtime-options/lib/browser";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { inject, injectable } from "inversify";
import { Configuration, CONFIGURATION_VALIDATOR_TYPE, ConfigurationProcessorServer } from "../../common";
import { ConfigurationDialogFactory } from "../configuration-dialog";
import { ConfigurationManager } from "../configuration-manager";
import { ConfigurationProcessorSelectionService } from "./configuration-processor-selection-service";

/*
 * TODO: Kill configuration when leaving the web page.
 */

export const ConfigurationProcessorService = Symbol("ConfigurationProcessorService");
export interface ConfigurationProcessorService {
    isConfigurationRunning(uuid: string): Promise<boolean>;
    getRunningConfigurations(): Promise<Configuration[]>;
    runConfiguration(configuration: Configuration): Promise<void>;
    killConfiguration(uuid: string): Promise<void>;
}

@injectable()
export class ConfigurationProcessorServiceImpl implements ConfigurationProcessorService {
    @inject(WorkspaceService) protected readonly workspace: WorkspaceService;
    @inject(ConfigurationProcessorServer) protected readonly server: ConfigurationProcessorServer;
    @inject(ConfigurationProcessorSelectionService) protected selection: ConfigurationProcessorSelectionService;
    @inject(ConfigurationManager) protected readonly manager: ConfigurationManager;
    @inject(ValidatorService) protected readonly validator: ValidatorService;
    @inject(ConfigurationDialogFactory) protected readonly dialogFactory: ConfigurationDialogFactory;

    public async runConfiguration(configuration: Configuration): Promise<void> {
        const workspace = await this.workspace.workspace;
        const context = { workspace: workspace ? workspace.uri : undefined };
        const errors = this.validator.validate(configuration.typeId, CONFIGURATION_VALIDATOR_TYPE, configuration.options);
        const keys = Object.keys(errors);
        const dialog = this.dialogFactory({ title: "Edit Configurations", selection: configuration });

        if (keys.length === 0) return this.server.runConfiguration(configuration, context);
        else await dialog.open();
    }

    public async isConfigurationRunning(uuid: string): Promise<boolean> {
        return this.server.isConfigurationRunning(uuid);
    }

    public async getRunningConfigurations(): Promise<Configuration[]> {
        const configurations = await this.manager.getConfigurations();
        const uuids = await this.server.getRunningConfigurations();

        return configurations.filter(configuration => uuids.indexOf(configuration.uuid) > -1);
    }

    public async killConfiguration(uuid: string): Promise<void> {
        return this.server.killConfiguration(uuid);
    }
}
