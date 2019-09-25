/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ComponentManager } from "./component-manager";
import { DynamicDialogElement } from "./dynamic-dialog";
import { ValueComponent, ValueComponentProps } from "./value-component";
import * as React from "react";
import RefObject = React.RefObject;

/**
 * Represents the props passed to a multi-value component.
 */
export interface MultiValueComponentProps extends ValueComponentProps {
    readonly manager: ComponentManager;
    readonly elements: DynamicDialogElement[];
}

/**
 * Abstract implementation of a multi-value component.
 */
export abstract class MultiValueComponent<P extends MultiValueComponentProps, S = {}> extends ValueComponent<P, S> {
    protected abstract get references(): RefObject<ValueComponent>[];

    public get valid(): boolean {
        const mapping = (reference: RefObject<ValueComponent>) => {
            const current = reference.current;

            if (current) return current.valid;
            else return false;
        };
        const reduction = (previous: boolean, current: boolean) => previous && current;

        return this.references.map(mapping).reduce(reduction, true);
    }

    public get filled(): boolean {
        return true;
    }
}
