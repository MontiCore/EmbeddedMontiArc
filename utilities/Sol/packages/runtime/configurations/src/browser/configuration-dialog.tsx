/*
 * (c) https://github.com/MontiCore/monticore
 */
import { OptionDialog } from "@embeddedmontiarc/sol-runtime-options/lib/browser";
import { OptionType } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { CircularProgress } from "@material-ui/core";
import { ConfirmDialog, DialogMode, DialogProps } from "@theia/core/lib/browser";
import { TreeItem, TreeView } from "@material-ui/lab";
import { ExpandMore, ChevronRight } from "@material-ui/icons";
import { bind } from "helpful-decorators";
import { inject, injectable, postConstruct } from "inversify";
import { ReactNode } from "react";
import { v4 } from "uuid";
import { CONFIGURATION_VALIDATOR_TYPE, Configuration } from "../common";

import * as React from "react";
import { ConfigurationManager } from "./configuration-manager";
import { ConfigurationType, ConfigurationTypeRegistry } from "./configuration-type-registry";

import Fragment = React.Fragment;
import ChangeEvent = React.ChangeEvent;

export interface ConfigurationDialogResult<V = any> { // tslint:disable-line:no-any
    readonly name: string;
    readonly options: V;
}

export interface ConfigurationDialogErrors<E = any> { // tslint:disable-line:no-any
    readonly name: string | undefined;
    readonly options: E;
}

export const ConfigurationDialogProps = Symbol("ConfigurationDialogProps");
export interface ConfigurationDialogProps extends DialogProps {
    readonly selection?: Configuration;
}

export const ConfigurationDialogFactory = Symbol("ConfigurationDialogFactory");
export interface ConfigurationDialogFactory {
    (props: ConfigurationDialogProps): ConfigurationDialog;
}

@injectable()
export class ConfigurationDialog extends OptionDialog<ConfigurationDialogResult | undefined> {
    @inject(ConfigurationManager) protected readonly manager: ConfigurationManager;
    @inject(ConfigurationTypeRegistry) protected readonly registry: ConfigurationTypeRegistry;

    protected configurations: Configuration[];
    protected types: ConfigurationType[];
    protected selectedItem: Configuration | ConfigurationType | undefined;
    protected name: string;
    protected errors: ConfigurationDialogErrors;

    public constructor(@inject(ConfigurationDialogProps) protected readonly props: ConfigurationDialogProps) {
        super(props);

        this.errors = { name: undefined, options: {} };
        this.selectedItem = this.props.selection;

        this.addClass("sol-runtime-configurations");
        this.addClass("dialog");
        this.appendAcceptButton("OK");
        this.appendCloseButton("Cancel");
    }

    @postConstruct()
    protected async init(): Promise<void> {
        await this.loadConfigurationTypes();

        this.toDispose.pushAll([
            this.manager.onConfigurationsAdded(() => this.updateConfigurations()),
            this.manager.onConfigurationRemoved(() => this.updateConfigurations()),
            this.manager.onConfigurationChanged(() => this.updateConfigurations())
        ]);

        return this.updateConfigurations();
    }

    public get value(): ConfigurationDialogResult | undefined {
        return { name: this.name, options: this.options };
    }

    protected async loadConfigurationTypes(): Promise<void> {
        this.types = await this.registry.getConfigurationTypes();

        return this.update();
    }

    protected async updateConfigurations(): Promise<void> {
        this.configurations = await this.manager.getConfigurations();

        return this.update();
    }

    protected getConfigurationType(id: string): ConfigurationType | undefined {
        if (this.types === undefined) return undefined;

        const index = this.types.findIndex(type => type.id === id);

        if (index > -1) return this.types[index];
    }

    protected getConfigurationTypeCategories(): { [category: string]: ConfigurationType[] } {
        const result = {} as { [category: string]: ConfigurationType[] };

        for (const type of this.types) {
            const category = type.category || "Default";

            result[category] = result[category] || [];

            result[category].push(type);
        }

        return result;
    }

    protected async selectItem(selectedItem: Configuration | ConfigurationType): Promise<void> {
        const data = this.getAsConfiguration(selectedItem);
        const name = data ? data.name : '';
        const options = data ? JSON.parse(JSON.stringify(data.options)) : undefined;

        this.selectedItem = selectedItem;
        this.name = name;
        this.options = options;

        return this.update();
    }

    protected isItemSelected(item: Configuration | ConfigurationType): boolean {
        const asConfiguration = this.getAsConfiguration(item);
        const asConfigurationType = this.getAsConfigurationType(item);
        const siConfiguration = this.getAsConfiguration(this.selectedItem);
        const siConfigurationType = this.getAsConfigurationType(this.selectedItem);

        if (asConfiguration && siConfiguration) return asConfiguration.uuid === siConfiguration.uuid;
        else if (asConfigurationType && siConfigurationType) return asConfigurationType.id === siConfigurationType.id;
        else return false;
    }

    protected getAsConfiguration(item: Configuration | ConfigurationType | undefined): Configuration | undefined {
        if (item && "uuid" in item) return item as Configuration;
    }

    protected getAsConfigurationType(item: Configuration | ConfigurationType | undefined): ConfigurationType | undefined {
        if (item && "id" in item) return item as ConfigurationType;
    }

    protected getIdentifierOf(item: Configuration | ConfigurationType | undefined): string {
        const selectedType = this.getAsConfigurationType(item);
        const selectedConfiguration = this.getAsConfiguration(item);

        return selectedType ? selectedType.id : (selectedConfiguration ? selectedConfiguration.uuid : v4());
    }

    protected getTypeOf(item: Configuration | ConfigurationType | undefined): ConfigurationType | undefined {
        const selectedType = this.getAsConfigurationType(item);
        const selectedConfiguration = this.getAsConfiguration(item);

        if (selectedType) return selectedType;
        else if (selectedConfiguration) return this.getConfigurationType(selectedConfiguration.typeId);
    }

    protected getOptionsOf(item: Configuration | ConfigurationType | undefined): OptionType[] {
        const type = this.getTypeOf(item);

        return type ? type.options : [];
    }

    protected getValuesOf(item: Configuration | ConfigurationType | undefined): any { // tslint:disable-line:no-any
        const selectedConfiguration = this.getAsConfiguration(item);

        if (selectedConfiguration) return selectedConfiguration.options;
    }

    protected async isValid(value: ConfigurationDialogResult, mode: DialogMode): Promise<boolean> {
        if (this.selectedItem === undefined) return true;

        const isValidName = this.validateName(value.name);
        const isValidOptions = await this.validateOptions(value.options);

        return isValidName && isValidOptions;
    }

    protected validateName(name: string): boolean {
        const isValidName = name.length > 0;

        if (isValidName) this.errors = { options: this.errors.options, name: undefined };
        else this.errors = { name: "A name for the configuration is required.", options: this.errors.options };

        return isValidName;
    }

    protected async validateOptions(options: any): Promise<boolean> { // tslint:disable-line:no-any
        const type = this.getTypeOf(this.selectedItem);

        if (type) return this.doValidateOptions(type, options);
        else return false;
    }

    protected async doValidateOptions(type: ConfigurationType, options: any): Promise<boolean> { // tslint:disable-line:no-any
        const errors = await this.validator.validate(type.id, CONFIGURATION_VALIDATOR_TYPE, options);
        const keys = Object.keys(errors as object);

        if (keys.length > 0) this.errors = { name: this.errors.name, options: errors };
        else this.errors = { name: this.errors.name, options: {} };

        return keys.length === 0;
    }

    protected async accept(): Promise<void> {
        const selectedConfiguration = this.getAsConfiguration(this.selectedItem);
        const selectedConfigurationType = this.getAsConfigurationType(this.selectedItem);

        await super.accept();

        if (selectedConfiguration) return this.setConfiguration(selectedConfiguration);
        else if (selectedConfigurationType) return this.addConfiguration(selectedConfigurationType);
    }

    protected async setConfiguration(configuration: Configuration): Promise<void> {
        return this.manager.setConfiguration(configuration.uuid, this.name, this.options);
    }

    protected async addConfiguration(type: ConfigurationType): Promise<void> {
        const uuid = v4();
        const configuration = { uuid: uuid, name: this.name, typeId: type.id, options: this.options };

        return this.manager.addConfiguration(configuration);
    }

    protected render(): ReactNode {
        return <div className="content-container">
            {this.renderLeftColumn()}
            {this.renderRightColumn()}
        </div>;
    }

    protected renderLoader(): ReactNode {
        return <div className="loader-container">
            <CircularProgress className="loader"/>
        </div>;
    }

    protected renderMessage(message: string): ReactNode {
        return <div className="message-container">
            <span className="message">{message}</span>
        </div>;
    }

    protected renderLeftColumn(): ReactNode {
        return <div className="left-column">
            {this.renderLeftColumnTopRow()}
            {this.renderLeftColumnBottomRow()}
        </div>;
    }

    protected renderLeftColumnTopRow(): ReactNode {
        return <div className="top-row">
            {this.configurations ? this.renderConfigurations() : this.renderLoader()}
        </div>;
    }

    protected renderConfigurations(): ReactNode {
        return this.configurations.length === 0 ? this.renderMessage("No Configurations") : this.doRenderConfigurations();
    }

    protected doRenderConfigurations(): ReactNode {
        return <div className="configurations">
            {this.configurations.map(configuration => this.renderConfiguration(configuration))}
        </div>;
    }

    protected renderConfiguration(configuration: Configuration): ReactNode {
        const className = this.isItemSelected(configuration) ? "configuration selected" : "configuration";

        return <div key={configuration.uuid} className={className}
                    onClick={() => this.onConfigurationClicked(configuration)}>
            {this.renderConfigurationIcon(configuration)}
            {this.renderConfigurationLabel(configuration)}
            {this.renderConfigurationDelete(configuration)}
        </div>;
    }

    protected renderConfigurationIcon(configuration: Configuration): ReactNode {
        const type = this.getConfigurationType(configuration.typeId);
        const iconClass = type ? `${type.iconClass} icon` : "fa fa-play icon";

        return <div className={iconClass}/>;
    }

    protected renderConfigurationLabel(configuration: Configuration): ReactNode {
        return <div className="label">{configuration.name}</div>;
    }

    protected renderConfigurationDelete(configuration: Configuration): ReactNode {
        return <div className="fa fa-trash delete" onClick={() => this.onConfigurationDeleteClicked(configuration)}/>;
    }

    protected renderLeftColumnBottomRow(): ReactNode {
        return <div className="bottom-row">
            {this.types ? this.renderConfigurationTypeCategories() : this.renderLoader()}
        </div>;
    }

    protected renderConfigurationTypeCategories(): ReactNode {
        return this.types.length === 0 ? this.renderMessage("No Configuration Types") : this.doRenderConfigurationTypeCategories();
    }

    protected doRenderConfigurationTypeCategories(): ReactNode {
        const categorizedTypes = this.getConfigurationTypeCategories();
        const categories = Object.keys(categorizedTypes);

        return <TreeView defaultCollapseIcon={<ExpandMore/>} defaultExpandIcon={<ChevronRight/>}>
            {categories.map(category => this.renderConfigurationTypeCategory(category))}
        </TreeView>;
    }

    protected renderConfigurationTypeCategory(category: string): ReactNode {
        const categorizedTypes = this.getConfigurationTypeCategories();
        const types = categorizedTypes[category];

        return <TreeItem key={category} nodeId={category} label={category}>
            {types.map(type => this.renderConfigurationType(type))}
        </TreeItem>;
    }

    protected renderConfigurationType(type: ConfigurationType): ReactNode {
        const className = this.isItemSelected(type) ? "configuration-type selected" : "configuration-type";

        return <TreeItem key={type.id} nodeId={type.id} label={this.doRenderConfigurationType(type)}
                         className={className} onClick={() => this.onConfigurationTypeClicked(type)}/>;
    }

    protected doRenderConfigurationType(type: ConfigurationType): ReactNode {
        return <Fragment>
            {this.renderConfigurationTypeIcon(type)}
            {this.renderConfigurationTypeLabel(type)}
        </Fragment>;
    }

    protected renderConfigurationTypeIcon(type: ConfigurationType): ReactNode {
        const iconClass = `${type.iconClass} icon`;

        return <div className={iconClass}/>;
    }

    protected renderConfigurationTypeLabel(type: ConfigurationType): ReactNode {
        return <div className="label">{type.label}</div>;
    }

    protected renderRightColumn(): ReactNode {
        return <div className="right-column">
            {this.selectedItem ? this.doRenderRightColumn() : this.renderMessage("No Selection")}
        </div>;
    }

    protected doRenderRightColumn(): ReactNode {
        return <Fragment>
            {this.renderRightColumnTopRow()}
            {this.renderRightColumnBottomRow()}
        </Fragment>;
    }

    protected renderRightColumnTopRow(): ReactNode {
        return <div className="top-row">
            {this.renderName()}
        </div>;
    }

    protected renderName(): ReactNode {
        const className = this.errors.name ? "name invalid" : "name";
        const key = this.getIdentifierOf(this.selectedItem);

        return <div key={key} className={className}>
            {this.renderNameInput()}
            {this.renderNameError()}
        </div>;
    }

    protected renderNameInput(): ReactNode {
        return <div className="value">
            <span className="label">Name*</span>
            <input type="text" className="form" defaultValue={this.name} onChange={this.onNameChange}/>
        </div>;
    }

    protected renderNameError(): ReactNode {
        return <div className="error">{this.errors.name}</div>;
    }

    protected renderRightColumnBottomRow(): ReactNode {
        const key = this.getIdentifierOf(this.selectedItem);
        const options = this.getOptionsOf(this.selectedItem);

        return <div className="bottom-row">
            {this.renderDynamicContent(key, options, this.errors.options, this.options)}
        </div>;
    }

    @bind
    protected async onNameChange(event: ChangeEvent<HTMLInputElement>): Promise<void> {
        this.name = event.target.value;

        return this.update();
    }

    @bind
    protected async onConfigurationClicked(configuration: Configuration): Promise<void> {
        return this.selectItem(configuration);
    }

    @bind
    protected async onConfigurationTypeClicked(type: ConfigurationType): Promise<void> {
        return this.selectItem(type);
    }

    @bind
    protected async onConfigurationDeleteClicked(configuration: Configuration): Promise<void> {
        const dialog = new ConfirmDialog({
            title: "Delete Configuration",
            msg: `Are you sure that you want to delete configuration ${configuration.name}?`
        });

        if (await dialog.open()) {
            this.selectedItem = undefined;

            return this.manager.removeConfiguration(configuration.uuid);
        }
    }
}
