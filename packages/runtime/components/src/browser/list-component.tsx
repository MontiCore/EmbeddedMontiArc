import { boundMethod } from "autobind-decorator";
import { interfaces } from "inversify";
import * as React from "react";
import { ComponentManager } from "./component-manager";
import { DynamicDialogElement } from "./dynamic-dialog";
import { GroupComponent } from "./group-component";
import { MultiValueComponent, MultiValueComponentProps } from "./multi-value-component";
import { ValueComponent } from "./value-component";
import * as uuid from "uuid";
import ReactNode = React.ReactNode;
import RefObject = React.RefObject;
import Container = interfaces.Container;

// tslint:disable:no-any

/**
 * Represents the props passed to a list item.
 */
export interface ListItemComponentProps extends MultiValueComponentProps {
    readonly index: number;

    onMinusClick(index: number): void;
}

/**
 * Implementation of a list item component.
 */
export class ListItemComponent extends ValueComponent<ListItemComponentProps> {
    protected readonly reference: RefObject<GroupComponent>;

    public constructor(props: ListItemComponentProps) {
        super(props);

        this.reference = React.createRef();
    }

    protected get index(): number {
        return this.props.index;
    }

    protected get manager(): ComponentManager {
        return this.props.manager;
    }

    protected get elements(): DynamicDialogElement[] {
        return this.props.elements;
    }

    public get value(): any {
        const current = this.reference.current;

        return current ? current.value : undefined;
    }

    public get valid(): boolean {
        const current = this.reference.current;

        return current ? current.valid : false;
    }

    public get filled(): boolean {
        const current = this.reference.current;

        return current ? current.filled : false;
    }

    public render(): ReactNode {
        return <div className="sol-runtime-components list-item">
            {this.renderGroup()}
            {this.renderButtonMinus()}
        </div>;
    }

    protected renderGroup(): ReactNode {
        return <div className="content">
            <GroupComponent ref={this.reference} manager={this.manager} widget={this.widget} elements={this.elements}
                            required={this.required}/>
        </div>;
    }

    protected renderButtonMinus(): ReactNode {
        return <div className="minus">
            <button className="theia-button secondary minus" onClick={this.onMinusClick}>-</button>
        </div>;
    }

    @boundMethod
    protected onMinusClick(): void {
        this.props.onMinusClick(this.index);
    }
}

/**
 * Represents the props passed to a list component.
 */
export interface ListComponentProps extends MultiValueComponentProps {
    readonly label: string;
}

/**
 * Represents the state of a list component.
 */
export interface ListComponentState {
    references: RefObject<ValueComponent>[];
    uuids: string[];
}

/**
 * Implementation of a list component.
 */
export class ListComponent extends MultiValueComponent<ListComponentProps, ListComponentState> {
    public constructor(props: ListComponentProps) {
        super(props);

        this.state = { references: [], uuids: [] };
    }

    protected get label(): string {
        return this.props.label;
    }

    protected get manager(): ComponentManager {
        return this.props.manager;
    }

    protected get elements(): DynamicDialogElement[] {
        return this.props.elements;
    }

    protected get references(): RefObject<ValueComponent>[] {
        return this.state.references;
    }

    protected get uuids(): string[] {
        return this.state.uuids;
    }

    public get value(): any[] {
        const result = [] as any[];

        this.references.forEach(reference => {
            const current = reference.current;

            if (current && current.filled) result.push(current.value);
        });

        return result;
    }

    protected updateState(references: RefObject<ValueComponent>[], uuids: string[]): void {
        this.setState({ references, uuids });
    }

    public render(): ReactNode {
        return <div className="sol-runtime-components component list-component">
            {this.renderButtonPlus()}
            {this.renderItems()}
        </div>;
    }

    protected renderButtonPlus(): ReactNode {
        return <div>
            <span className="label">{this.label}</span>
            <button className="theia-button secondary plus" onClick={this.onPlusClick}>+</button>
        </div>;
    }

    protected renderItems(): ReactNode {
        return this.references.map((value, index) => {
            const reference = this.references[index] as RefObject<ListItemComponent>;

            return <ListItemComponent key={this.uuids[index]} index={index} ref={reference} {...this.props}
                                      onMinusClick={this.onMinusClick}
            />;
        });
    }

    @boundMethod
    protected onPlusClick(): void {
        this.references.push(React.createRef());
        this.uuids.push(uuid.v4());
        this.updateState(this.references, this.uuids);
    }

    @boundMethod
    protected onMinusClick(index: number): void {
        this.references.splice(index, 1);
        this.uuids.splice(index, 1);
        this.updateState(this.references, this.uuids);
    }

    public static readonly TYPE: string = "list";

    public static createComponent(ref: RefObject<ListComponent>, props: ListComponentProps, container: Container): ReactNode {
        const manager = container.get<ComponentManager>(ComponentManager);

        return <ListComponent ref={ref} manager={manager} {...props}/>;
    }
}
