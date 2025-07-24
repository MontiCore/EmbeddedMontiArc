/* (c) https://github.com/MontiCore/monticore */
import {Module} from "../se.rwth.common.module/module";

/*export abstract class UIElement {
    protected element: JQuery;

    protected constructor(attributes: any = {}) {
        this.createElement(attributes);
        this.extendAttributes();
    }

    protected abstract createElement(attributes: any): void;

    protected abstract extendAttributes(): void;


    public setAttribute(name: string, value: string | number) {
        this.element.attr(name, value);
    }

    public getAttribute(name: string): string | number {
        return this.element.attr(name);
    }

    public appendChild(element: UIElement): void {
        const $element = element.element;

        this.element.append($element);
    }

    public insertBefore(element: UIElement, selector: string): void {
        const $element = element.element;
        const $sibling = this.element.find(selector);

        $sibling.before($element);
    }

    public removeChild(element: UIElement): void {
        const $element = element.element;
        const $node = this.element.find($element);

        $node.remove();
    }

    public addEventListener(eventName: string, callback: Callback): void {
        this.element.on(eventName, callback);
    }
}

export abstract class Container extends UIElement {
    protected createElement(attributes: any): void {
        this.element = jQuery("<div/>", attributes);
    }

    protected extendAttributes(): void {
        this.element.addClass("container");
    }


    public show(): void {
        this.element.show();
    }

    public hide(): void {
        this.element.hide();
    }
}

export abstract class Panel extends Container {
    protected extendAttributes(): void {
        super.extendAttributes();
        this.element.addClass("panel");
    }
}

export abstract class List extends UIElement {
    protected createElement(attributes: any): void {
        this.element = jQuery("<ul/>", attributes);
    }

    protected extendAttributes(): void {}
}

export abstract class ListItem extends UIElement {
    protected createElement(attributes: any): void {
        this.element = jQuery("<li/>", attributes);
    }

    protected extendAttributes(): void {}
}

export abstract class Button extends UIElement {
    protected createElement(attributes: any): void {

    }
}

export abstract class Label extends UIElement {

}

export abstract class TextBox extends UIElement {

}*/

export class UI extends Module {
    protected constructor() {
        super("common.ui");
    }

    protected setDependencies(): void {}

    protected static instance: UI;

    protected static getInstance(): UI {
        return UI.instance = UI.instance ? UI.instance : new UI();
    }

    public static insertHTML($parent: JQuery, html: string, module: Module): void {
        return UI.getInstance().doInsertHTML($parent, html, module);
    }

    protected doInsertHTML($parent: JQuery, html: string, module: Module): void {
        const $child = jQuery(html);

        $parent.append($child);
        module.on("unload", $child.remove);
    }

    public static insertCSS(css: string, module: Module): void {
        UI.getInstance().doInsertCSS(css, module);
    }

    protected doInsertCSS(css: string, module: Module): void {
        const $link = jQuery("<link/>", { href: css, type: "text/css", rel: "stylesheet" });

        jQuery("head").append($link);
        module.on("unload", $link.remove);
    }
}
