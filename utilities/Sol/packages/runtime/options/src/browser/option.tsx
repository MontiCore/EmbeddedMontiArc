/*
 * (c) https://github.com/MontiCore/monticore
 */

// tslint:disable:no-any

import { Component, ReactNode } from "react";
import { OptionType } from "../common";
import { OptionManager } from "./option-manager";

import * as React from "react";

export interface OptionProps<R, E = string> {
    readonly value: R | undefined;
    readonly error: E | undefined;

    setParentValue(value: R): void;
}

export interface CompositeOptionProps<R, E> extends OptionProps<R, E> {
    readonly options: OptionType[];
    readonly manager: OptionManager;
}

export interface ArrayOptionProps extends CompositeOptionProps<any[], any[]> {
    /* Placeholder */
}

export interface ObjectOptionProps extends CompositeOptionProps<any[string], any[string]> {
    /* Placeholder */
}

export abstract class Option<R = any, E = string, P extends OptionProps<R, E> = OptionProps<R, E>, S = {}> extends Component<P, S> {
    protected readonly defaultError: E | undefined;

    protected value: R;

    public constructor(props: P, defaultValue: R, defaultError?: E) {
        super(props);

        this.value = props.value || defaultValue;
        this.defaultError = defaultError;
    }

    public get error(): E | undefined {
        return this.props.error || this.defaultError;
    }
}

export abstract class ReturnsOption<R, P extends OptionProps<R, string> = OptionProps<R, string>, S = {}> extends Option<R, string, P, S> {
    protected setValue(value: R): void {
        this.value = value;

        this.props.setParentValue(value);
    }

    protected renderError(): ReactNode {
        return <div className="error">{this.error}</div>;
    }
}

export abstract class CompositeOption<R, E, P extends CompositeOptionProps<R, E> = CompositeOptionProps<R, E>, S = {}> extends Option<R, E, P, S> {
    protected get options(): OptionType[] {
        return this.props.options;
    }

    protected get manager(): OptionManager {
        return this.props.manager;
    }
}

export abstract class ArrayOption<
            P extends ArrayOptionProps = ArrayOptionProps,
            S = {}
        > extends CompositeOption<any[], any[number], P, S> {
    protected constructor(props: P) {
        super(props, [], []);
    }

    protected setValue(index: number, value: any): void {
        this.value[index] = value;

        this.props.setParentValue(this.value);
    }

    protected addValue(value: any): void {
        this.value.push(value);

        this.props.setParentValue(this.value);
    }

    protected removeValue(index: number): void {
        this.value.splice(index, 1);

        this.props.setParentValue(this.value);
    }
}

export abstract class ObjectOption<
            P extends ObjectOptionProps = ObjectOptionProps,
            S = {}
        > extends CompositeOption<any[string], any[string], P, S> {
    public constructor(props: P) {
        super(props, {}, {});
    }

    protected setValue(name: string, value: any): void {
        this.value[name] = value;

        this.props.setParentValue(this.value);
    }
}
