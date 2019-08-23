import { AbstractDialog, DialogMode, DialogProps } from "@theia/core/lib/browser";
import { ComponentManager } from "./component-manager";
import { GroupComponent } from "./group-component";
import * as React from "react";
import ReactNode = React.ReactNode;
import RefObject = React.RefObject;

// tslint:disable:no-any

/**
 * Represents an element which is later passed to a dynamic dialog.
 */
export interface DynamicDialogElement {
    readonly type: string;
    readonly props: any;
    readonly variable: string;
}

/**
 * Represents the props passed to a dynamic dialog.
 */
export class DynamicDialogProps extends DialogProps {
    readonly elements: DynamicDialogElement[];
}

/**
 * Abstract implementation of a dynamic dialog.
 */
export abstract class DynamicDialog extends AbstractDialog<any[string]> {
    protected readonly props: DynamicDialogProps;
    protected readonly manager: ComponentManager;
    protected readonly reference: RefObject<GroupComponent>;

    protected constructor(manager: ComponentManager, props: DynamicDialogProps) {
        super(props);

        this.manager = manager;
        this.reference = React.createRef();
    }

    protected get elements(): DynamicDialogElement[] {
        const mapping = (element: DynamicDialogElement) => ({
            type: element.type,
            variable: element.variable,
            props: { ...element.props, widget: this }
        });

        return this.props.elements.map(mapping);
    }

    public get value(): any[string] {
        const current = this.reference.current;

        return current ?  current.value : {};
    }

    protected isValid(value: any[string], mode: DialogMode): string {
        const current = this.reference.current;

        return current && current.valid ? '' : "There are erroneous form fields.";
    }

    protected renderDynamicContent(): ReactNode {
        return <div className="sol-runtime-components content">
            <GroupComponent ref={this.reference} widget={this} manager={this.manager} elements={this.elements}
                            required={false}/>
        </div>;
    }
}
