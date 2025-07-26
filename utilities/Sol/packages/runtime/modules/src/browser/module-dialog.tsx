/*
 * (c) https://github.com/MontiCore/monticore
 */
import { OptionDialog } from "@embeddedmontiarc/sol-runtime-options/lib/browser";
import { ChevronRight, ExpandMore } from "@material-ui/icons";
import { TreeItem, TreeView } from "@material-ui/lab";
import { DialogProps, DialogMode } from "@theia/core/lib/browser";
import { bind } from "helpful-decorators";
import { inject, injectable, postConstruct } from "inversify";
import { ReactNode } from "react";
import { MODULE_VALIDATOR_TYPE } from "../common";
import { CircularProgress } from "@material-ui/core";
import { ModuleType, ModuleTypeRegistry } from "./module-type-registry";

import * as React from "react";

export const ModuleDialogProps = Symbol("ModuleDialogProps");
export interface ModuleDialogProps extends DialogProps {
    readonly destination: string;
}

export const ModuleDialogFactory = Symbol("ModuleDialogFactory");
export interface ModuleDialogFactory {
    (props: ModuleDialogProps): ModuleDialog;
}

@injectable()
export class ModuleDialog extends OptionDialog<string> {
    @inject(ModuleTypeRegistry) protected readonly registry: ModuleTypeRegistry;

    protected types: { [category: string]: ModuleType[] };
    protected typeId: string;

    public constructor(@inject(ModuleDialogProps) protected readonly props: ModuleDialogProps) {
        super(props);

        this.appendAcceptButton("OK");
        this.appendCloseButton("Cancel");
    }

    @postConstruct()
    protected async init(): Promise<void> {
        const types = await this.registry.getModuleTypes();

        this.types = this.getModuleTypesCategorized(types);

        this.update();
    }

    protected getModuleTypesCategorized(types: ModuleType[]): { [category: string]: ModuleType[] } {
        const categories = {} as { [category: string]: ModuleType[] };

        for (const type of types) {
            const category = type.category || "Default";

            categories[category] = categories[category] || [];

            categories[category].push(type);
        }

        return categories;
    }

    public get value(): string {
        return this.typeId;
    }

    protected async isValid(value: string, mode: DialogMode): Promise<boolean> {
        if (this.typeId === undefined) return false;

        return this.validator.validate(this.typeId, MODULE_VALIDATOR_TYPE, this.props.destination);
    }

    protected render(): ReactNode {
        return <div className="sol-runtime-modules dialog">
            {this.renderCategories()}
        </div>;
    }

    protected renderLoader(): ReactNode {
        return <div className="loader-container">
            <CircularProgress className="loader"/>
        </div>;
    }

    protected renderMessage(): ReactNode {
        return <div className="message-container">
            <span className="message">No Module Types</span>
        </div>;
    }

    protected renderCategories(): ReactNode {
        return <div className="categories">
            {this.types ? this.doRenderCategories() : this.renderLoader()}
        </div>;
    }

    protected doRenderCategories(): ReactNode {
        const categories = Object.keys(this.types);

        if (categories.length === 0) return this.renderMessage();

        return <TreeView defaultCollapseIcon={<ExpandMore/>} defaultExpandIcon={<ChevronRight/>}>
            {categories.map(category => this.renderCategory(category))}
        </TreeView>;
    }

    protected renderCategory(category: string): ReactNode {
        const types = this.types[category];

        return <TreeItem key={category} nodeId={category} label={category}>
            {types.map(type => this.renderModuleType(type))}
        </TreeItem>;
    }

    protected renderModuleType(type: ModuleType): ReactNode {
        const className = this.typeId === type.id ? "module-type selected" : "module-type";

        return <div key={type.id} className={className} onClick={() => this.onModuleTypeClicked(type)}>
            {this.renderModuleTypeIcon(type)}
            {this.renderModuleTypeLabel(type)}
        </div>;
    }

    protected renderModuleTypeIcon(type: ModuleType): ReactNode {
        const iconClass = `${type.iconClass} icon`;

        return <div className={iconClass}/>;
    }

    protected renderModuleTypeLabel(type: ModuleType): ReactNode {
        return <div className="label">{type.label}</div>;
    }

    @bind
    protected async onModuleTypeClicked(type: ModuleType): Promise<void> {
        this.typeId = type.id;

        return this.update();
    }
}
