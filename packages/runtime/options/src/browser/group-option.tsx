/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ObjectOption, ObjectOptionProps } from "./option";

import * as React from "react";

import ReactNode = React.ReactNode;

/**
 * Implementation of a group component.
 */
export class GroupOption extends ObjectOption<ObjectOptionProps> {
    public render(): ReactNode {
        return <div className="sol-runtime-options component group-option">
            {this.renderElements()}
        </div>;
    }

    protected renderElements(): ReactNode {
        return this.options.map(option => this.manager.renderComponent(option.type, {
            key: option.name,
            value: this.value[option.name],
            error: this.error[option.name],
            setParentValue: value => this.setValue(option.name, value),
            ...option.props
        }));
    }
}
