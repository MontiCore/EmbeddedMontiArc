/* (c) https://github.com/MontiCore/monticore */
import { ComponentManager } from "./component-manager";
import { DynamicDialogElement } from "./dynamic-dialog";
import { MultiValueComponent, MultiValueComponentProps } from "./multi-value-component";
import { ValueComponent } from "./value-component";
import * as React from "react";
import ReactNode = React.ReactNode;
import RefObject = React.RefObject;

// tslint:disable:no-any

/**
 * Implementation of a group component.
 */
export class GroupComponent extends MultiValueComponent<MultiValueComponentProps> {
    protected readonly references: RefObject<ValueComponent>[];

    public constructor(props: MultiValueComponentProps) {
        super(props);

        this.references = [];

        this.initReferences();
    }

    protected initReferences(): void {
        const consumer = () => this.references.push(React.createRef());

        this.elements.forEach(consumer);
    }

    protected get manager(): ComponentManager {
        return this.props.manager;
    }

    protected get elements(): DynamicDialogElement[] {
        return this.props.elements;
    }

    public get value(): any[string] {
        const result = {} as any[string];

        this.elements.forEach((element: DynamicDialogElement, index: number) => {
            const reference = this.references[index].current;

            if (reference && reference.filled) result[element.variable] = reference.value;
        });

        return result;
    }

    public render(): ReactNode {
        return <div className="sol-runtime-components component group-component">
            {this.renderElements()}
        </div>;
    }

    protected renderElements(): ReactNode {
        return this.elements.map((element: DynamicDialogElement, index: number) =>
            this.manager.renderComponent(element.type, this.references[index], { ...element.props, widget: this.widget }));
    }
}
