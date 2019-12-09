/*
 * (c) https://github.com/MontiCore/monticore
 */

// tslint:disable:no-any

import { bind } from "helpful-decorators";
import { interfaces } from "inversify";
import { OptionType } from "../common";
import { OptionManager } from "./option-manager";
import { GroupOption } from "./group-option";
import { ArrayOption, ArrayOptionProps, ObjectOptionProps, ReturnsOption } from "./option";

import * as React from "react";
import * as uuid from "uuid";

import ReactNode = React.ReactNode;
import Container = interfaces.Container;

/**
 * Represents the props passed to a list item.
 */
export interface ListItemProps extends ObjectOptionProps {
    readonly index: number;

    onMinusClick(index: number): void;
}

/**
 * Implementation of a list item component.
 */
export class ListItem extends ReturnsOption<any[string], ListItemProps> {
    protected get manager(): OptionManager {
        return this.props.manager;
    }

    protected get options(): OptionType[] {
        return this.props.options;
    }

    protected get index(): number {
        return this.props.index;
    }

    public render(): ReactNode {
        return <div className="sol-runtime-options list-item">
            {this.renderGroup()}
            {this.renderButtonMinus()}
        </div>;
    }

    protected renderGroup(): ReactNode {
        return <div className="content">
            <GroupOption value={this.value} error={this.error} manager={this.manager} options={this.options}
                         setParentValue={value => this.setValue(value)}/>
        </div>;
    }

    protected renderButtonMinus(): ReactNode {
        return <div className="minus">
            <button className="theia-button secondary minus" onClick={this.onMinusClick}>-</button>
        </div>;
    }

    @bind
    protected onMinusClick(): void {
        this.props.onMinusClick(this.index);
    }
}

/**
 * Represents the props passed to a list component.
 */
export interface ListOptionProps extends ArrayOptionProps {
    readonly label: string;
}

/**
 * Implementation of a list component.
 */
export class ListOption extends ArrayOption<ListOptionProps> {
    protected readonly uuids: string[];

    public constructor(props: ListOptionProps) {
        super(props);

        this.uuids = [];
    }

    protected get label(): string {
        return this.props.label;
    }

    public render(): ReactNode {
        return <div className="sol-runtime-options component list-option">
            {this.renderButtonPlus()}
            {this.renderItems()}
        </div>;
    }

    protected renderButtonPlus(): ReactNode {
        return <div>
            <span className="label">{this.label}</span>
            <button className="theia-button secondary plus" onClick={this.onPlusClick}>+</button>
        </div>;
    }

    protected renderItems(): ReactNode {
        return this.value.map((id: string, index: number) =>
            <ListItem key={this.uuids[index]} index={index} options={this.options} manager={this.manager}
                      error={this.error[index]} value={this.value[index]}
                      setParentValue={value => this.setValue(index, value)} onMinusClick={this.onMinusClick}/>
        );
    }

    @bind
    protected onPlusClick(): void {
        this.uuids.push(uuid.v4());
        this.addValue({});
    }

    @bind
    protected onMinusClick(index: number): void {
        this.uuids.splice(index, 1);
        this.removeValue(index);
    }

    public static readonly TYPE: string = "de.monticore.lang.monticar.sol.option.types.List";

    public static createComponent(props: ListOptionProps, container: Container): ReactNode {
        const manager = container.get<OptionManager>(OptionManager);

        return <ListOption manager={manager} {...props}/>;
    }
}
