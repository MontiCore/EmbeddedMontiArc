/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ContributionProvider } from "@theia/core";
import { inject, injectable, named, postConstruct } from "inversify";
import { ValueComponent, ValueComponentProps } from "./value-component";
import * as React from "react";
import ReactNode = React.ReactNode;
import RefObject = React.RefObject;

// tslint:disable:no-any

export const ComponentFactory = Symbol("ComponentFactory");
/**
 * An interface to be implemented when registering a ComponentFactory for a certain type and which renders
 * a component on given arguments.
 */
export interface ComponentFactory {
    /**
     * The type which is handled by this ComponentFactory. Must be unique amongst peers.
     */
    readonly type: string;

    /**
     * Renders the component registered under the given type.
     * @param ref A reference to the component to be rendered and which is used to retrieve the values.
     * @param props The props to be handed to the newly rendered component.
     */
    renderComponent<P extends ValueComponentProps>(ref: RefObject<ValueComponent>, props?: P): ReactNode;
}

export const ComponentManager = Symbol("ComponentManager");
/**
 * An interface to be implemented by the class responsible for the management of the ComponentFactories and
 * which delegates rendering to a concrete ComponentFactory.
 */
export interface ComponentManager {
    /**
     * Renders the component registered under the given type.
     * @param type The type of component to be rendered.
     * @param ref A reference to the component to be rendered and which is used to retrieve the values.
     * @param props The props to be handed to the newly rendered component.
     */
    renderComponent<P extends ValueComponentProps>(type: string, ref: RefObject<ValueComponent>, props?: P): ReactNode;
}

@injectable()
export class ComponentManagerImpl implements ComponentManager {
    @inject(ContributionProvider) @named(ComponentFactory)
    protected readonly contributions: ContributionProvider<ComponentFactory>;

    protected factories: Map<string, ComponentFactory>;

    @postConstruct()
    protected init(): void {
        this.factories = new Map();

        this.initFactories();
    }

    protected initFactories(): void {
        const contributions = this.contributions.getContributions();

        contributions.forEach(contribution => this.factories.set(contribution.type, contribution));
    }

    public renderComponent<P extends ValueComponentProps>(type: string, ref: RefObject<ValueComponent>, props?: P): ReactNode {
        const factory = this.factories.get(type);

        if (factory) return factory.renderComponent(ref, props);
        else throw new Error(`No ComponentFactory registered under the ID of ${type}.`);
    }
}
