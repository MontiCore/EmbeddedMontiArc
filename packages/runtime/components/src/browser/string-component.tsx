/*
 * (c) https://github.com/MontiCore/monticore
 */
import { boundMethod } from "autobind-decorator";
import { ChangeEvent } from "react";
import * as React from "react";
import { ValueComponent, ValueComponentProps } from "./value-component";
import ReactNode = React.ReactNode;
import RefObject = React.RefObject;

/**
 * Represents the props passed to a string component.
 */
export interface StringComponentProps extends ValueComponentProps {
    readonly label: string;
    readonly defaultValue?: string;
}

/**
 * Represents the state of a string component.
 */
export interface StringComponentState {
    value: string;
}

/**
 * Implementation of a string component.
 */
export class StringComponent extends ValueComponent<StringComponentProps, StringComponentState> {
    public constructor(props: StringComponentProps) {
        super(props);

        this.state = { value: props.defaultValue || '' };
    }

    protected get label(): string {
        return `${this.props.label}${this.required ? '*' : ''}`;
    }

    public get filled(): boolean {
        return this.value.length > 0;
    }

    public get value(): string {
        return this.state.value;
    }

    protected get className(): string {
        const invalid = this.valid ? '' : "invalid";

        return `sol-runtime-components component ${invalid} string-component`;
    }

    public render(): ReactNode {
        return <div className={this.className}>
            <span className="label">{this.label}</span>
            <input type="text" value={this.value} onChange={this.onChange}/>
        </div>;
    }

    @boundMethod
    protected onChange(event: ChangeEvent<HTMLInputElement>): void {
        this.setState({ value: event.target.value });
    }

    public static readonly TYPE: string = "string";

    public static createComponent(ref: RefObject<StringComponent>, props: StringComponentProps): ReactNode {
        return <StringComponent ref={ref} {...props}/>;
    }
}
