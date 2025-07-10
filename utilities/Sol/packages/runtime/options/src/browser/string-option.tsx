/*
 * (c) https://github.com/MontiCore/monticore
 */
import { bind } from "helpful-decorators";
import { ChangeEvent } from "react";
import { OptionProps, ReturnsOption } from "./option";

import * as React from "react";

import ReactNode = React.ReactNode;

/**
 * Represents the props passed to a string component.
 */
export interface StringOptionProps extends OptionProps<string> {
    readonly label: string;
    readonly required: boolean;
}

/**
 * Implementation of a string component.
 */
export class StringOption extends ReturnsOption<string, StringOptionProps> {
    public constructor(props: StringOptionProps) {
        super(props, '');
    }

    protected get label(): string {
        return `${this.props.label}${this.props.required ? '*' : ''}`;
    }

    protected get className(): string {
        return `sol-runtime-options component ${this.error ? "invalid" : ''} string-option`;
    }

    public render(): ReactNode {
        return <div className={this.className}>
            {this.renderValue()}
            {this.renderError()}
        </div>;
    }

    protected renderValue(): ReactNode {
        return <div className="value">
            <span className="label">{this.label}</span>
            <input type="text" defaultValue={this.value} onChange={this.onChange}/>
        </div>;
    }

    @bind
    protected onChange(event: ChangeEvent<HTMLInputElement>): void {
        this.setValue(event.target.value);
    }

    public static readonly TYPE: string = "de.monticore.lang.monticar.sol.option.types.String";

    public static createComponent(props: StringOptionProps): ReactNode {
        return <StringOption {...props}/>;
    }
}
