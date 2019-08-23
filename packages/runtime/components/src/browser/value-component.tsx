import * as React from "react";
import { Widget } from "@theia/core/lib/browser";
import Component = React.Component;

// tslint:disable:no-any

/**
 * Represents the props passed to a value component which returns one or more values.
 */
export interface ValueComponentProps {
    readonly required: boolean;
    readonly widget?: Widget;
}

/**
 * Abstract implementation of a ValueComponent.
 */
export abstract class ValueComponent<P extends ValueComponentProps = { required: false }, S = {}> extends Component<P, S> {
    protected get required(): boolean {
        return this.props.required;
    }

    protected get widget(): Widget | undefined {
        return this.props.widget;
    }

    public get valid(): boolean {
        return (this.required && this.filled) || !this.required;
    }

    public get filled(): boolean {
        return true;
    }

    public abstract get value(): any;

    public componentDidUpdate(): void {
        if (this.widget) this.widget.update();
    }
}
