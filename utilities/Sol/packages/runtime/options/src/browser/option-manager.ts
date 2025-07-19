/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ContributionProvider } from "@theia/core";
import { inject, injectable, named, postConstruct } from "inversify";
import { OptionProps } from "./option";

import * as React from "react";

import ReactNode = React.ReactNode;

export const OptionFactory = Symbol("OptionFactory");
/**
 * An interface to be implemented when registering a ComponentFactory for a certain type and which renders
 * a component on given arguments.
 */
export interface OptionFactory {
    /**
     * The type which is handled by this ComponentFactory. Must be unique amongst peers.
     */
    readonly type: string;

    /**
     * Renders the component registered under the given type.
     * @param props The props to be handed to the newly rendered component.
     */
    renderComponent<R, E, P extends OptionProps<R, E>>(props?: P): ReactNode;
}

export const OptionManager = Symbol("OptionManager");
/**
 * An interface to be implemented by the class responsible for the management of the ComponentFactories and
 * which delegates rendering to a concrete ComponentFactory.
 */
export interface OptionManager {
    /**
     * Renders the component registered under the given type.
     * @param type The type of component to be rendered.
     * @param props The props to be handed to the newly rendered component.
     */
    renderComponent<R, E, P extends OptionProps<R, E>>(type: string, props?: P): ReactNode;
}

@injectable()
export class OptionManagerImpl implements OptionManager {
    @inject(ContributionProvider) @named(OptionFactory)
    protected readonly contributions: ContributionProvider<OptionFactory>;

    protected factories: Map<string, OptionFactory>;

    @postConstruct()
    protected init(): void {
        this.factories = new Map();

        this.initFactories();
    }

    protected initFactories(): void {
        const contributions = this.contributions.getContributions();

        contributions.forEach(contribution => this.factories.set(contribution.type, contribution));
    }

    public renderComponent<R, E, P extends OptionProps<R, E>>(type: string, props?: P): ReactNode {
        const factory = this.factories.get(type);

        if (factory) return factory.renderComponent(props);
        else throw new Error(`No ComponentFactory registered under the ID of ${type}.`);
    }
}
