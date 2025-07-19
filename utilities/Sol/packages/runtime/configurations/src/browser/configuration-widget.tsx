/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CircularProgress } from "@material-ui/core";
import { Message, ReactWidget } from "@theia/core/lib/browser";
import { bind } from "helpful-decorators";
import { inject, injectable, postConstruct } from "inversify";
import { ReactNode } from "react";
import { Configuration, ConfigurationProcessorServer, ConfigurationProcessorWatcher } from "../common";
import { ConfigurationManager } from "./configuration-manager";
import { ConfigurationType, ConfigurationTypeRegistry } from "./configuration-type-registry";
import { ConfigurationProcessorSelectionService, ConfigurationProcessorService } from "./processor";

import * as React from "react";

/*
 * TODO:
 *  * Right-Click > Edit Configuration should open the configuration-dialog for the right-clicked configuration.
 *  * Add grouping of configurations in the case of multiple workspace roots.
 */

@injectable()
export class ConfigurationWidget extends ReactWidget {
    @inject(ConfigurationManager) protected readonly manager: ConfigurationManager;
    @inject(ConfigurationTypeRegistry) protected readonly registry: ConfigurationTypeRegistry;
    @inject(ConfigurationProcessorWatcher) protected readonly watcher: ConfigurationProcessorWatcher;
    @inject(ConfigurationProcessorServer) protected readonly server: ConfigurationProcessorServer;
    @inject(ConfigurationProcessorService) protected readonly service: ConfigurationProcessorService;

    @inject(ConfigurationProcessorSelectionService)
    protected readonly selectionService: ConfigurationProcessorSelectionService;

    protected readonly types: Map<string, ConfigurationType>;

    protected configurations: Configuration[];
    protected runningConfigurations: string[];
    protected selectedConfiguration: Configuration | undefined;

    public constructor() {
        super();

        this.id = ConfigurationWidget.ID;
        this.title.label = "Configurations";
        this.title.caption = "Configurations";
        this.title.closable = true;
        this.title.iconClass = "fa fa-play";

        this.types = new Map();

        this.node.tabIndex = -1;
    }

    @postConstruct()
    protected async init(): Promise<void> {
        await this.loadConfigurationTypes();

        this.toDispose.pushAll([
            this.manager.onConfigurationsAdded(() => this.updateConfigurations()),
            this.manager.onConfigurationChanged(() => this.updateConfigurations()),
            this.manager.onConfigurationRemoved(() => this.updateConfigurations()),
            this.watcher.onConfigurationStarted(() => this.updateConfigurations()),
            this.watcher.onConfigurationExited(() => this.updateConfigurations())
        ]);

        this.update();
        return this.updateConfigurations();
    }

    protected async loadConfigurationTypes(): Promise<void> {
        const types = await this.registry.getConfigurationTypes();

        types.forEach(type => this.types.set(type.id, type));
    }

    protected async updateConfigurations(): Promise<void> {
        const configurations = await this.manager.getConfigurations();

        this.runningConfigurations = await this.server.getRunningConfigurations();
        this.configurations = this.renameDuplicates(configurations);

        this.update();
    }

    protected renameDuplicates(configurations: Configuration[]): Configuration[] {
        const renamedConfigurations = [] as Configuration[];

        for (const configuration of configurations) {
            const index = renamedConfigurations.findIndex(c => c.name === configuration.name);

            if (index > -1) renamedConfigurations.push(this.renameDuplicate(renamedConfigurations, configuration, 2));
            else renamedConfigurations.push(configuration);
        }

        return renamedConfigurations;
    }

    protected renameDuplicate(configurations: Configuration[], configuration: Configuration, suffix: number): Configuration {
        const suffixedName = `${configuration.name} (${suffix})`;
        const index = configurations.findIndex(c => c.name === suffixedName);

        if (index > -1) return this.renameDuplicate(configurations, configuration, suffix + 1);
        else return { ...configuration, name: suffixedName };
    }

    protected selectConfiguration(configuration: Configuration): void {
        this.selectedConfiguration = configuration;

        this.update();
    }

    protected isConfigurationSelected(configuration: Configuration): boolean {
        return (this.selectedConfiguration && this.selectedConfiguration.uuid === configuration.uuid) || false;
    }

    protected isConfigurationRunning(configuration: Configuration): boolean {
        return this.runningConfigurations.indexOf(configuration.uuid) > -1;
    }

    protected getConfigurationClassName(configuration: Configuration): string {
        const selected = this.isConfigurationSelected(configuration) ? "selected" : "";
        const running = this.isConfigurationRunning(configuration) ? "running" : "";

        return `configuration ${selected} ${running}`;
    }

    protected render(): ReactNode {
        return <div className="sol-runtime-configurations widget">
            {this.configurations ? this.renderContent() : this.renderLoader()}
        </div>;
    }

    protected renderLoader(): ReactNode {
        return <div className="loader-container">
            <CircularProgress className="loader"/>
        </div>;
    }

    protected renderContent(): ReactNode {
        return this.configurations.length === 0 ? this.renderMessage() : this.renderConfigurations();
    }

    protected renderMessage(): ReactNode {
        return <div className="message-container">
            <span className="message">No Configurations</span>
        </div>;
    }

    protected renderConfigurations(): ReactNode {
        return <div className="configurations">
            {this.configurations.map(configuration => this.renderConfiguration(configuration))}
        </div>;
    }

    protected renderConfiguration(configuration: Configuration): ReactNode {
        const className = this.getConfigurationClassName(configuration);

        return <div key={configuration.uuid} className={className}
                    onClick={() => this.onConfigurationClicked(configuration)}
                    onDoubleClick={() => this.onConfigurationDoubleClicked(configuration)}>
            {this.renderIcon(configuration)}
            <div className="label">{configuration.name}</div>
        </div>;
    }

    protected renderIcon(configuration: Configuration): ReactNode {
        const type = this.types.get(configuration.typeId);
        const iconClass = type ? `${type.iconClass} icon` : "fa fa-play icon";

        return <div className={iconClass}/>;
    }

    protected onActivateRequest(message: Message): void {
        super.onActivateRequest(message);
        this.node.focus();
    }

    @bind
    protected async onConfigurationClicked(configuration: Configuration): Promise<void> {
        this.selectConfiguration(configuration);
    }

    @bind
    protected async onConfigurationDoubleClicked(configuration: Configuration): Promise<void> {
        if (this.isConfigurationRunning(configuration)) return this.selectionService.select(configuration);
        else return this.service.runConfiguration(configuration);
    }
}

export namespace ConfigurationWidget {
    export const ID: string = "configurations";
}
