/* (c) https://github.com/MontiCore/monticore */
export type Callback = (...args: any[]) => void;

export abstract class EventBus {
    protected events: any;

    protected constructor() {
        this.events = {};
    }

    public on(eventName: string, callback: Callback): void {
        jQuery(this.events).on(eventName, callback);
    }

    public off(eventName: string, callback: Callback): void {
        jQuery(this.events).off(eventName, callback);
    }

    public once(eventName: string, callback: Callback): void {
        jQuery(this.events).one(eventName, callback);
    }

    public emit(eventName: string, ...args): void {
        jQuery(this.events).trigger(eventName, args);
    }

    public static on(id: string, event: string, callback: Callback): void {
        const module = Module.get(id);

        if(module) module.on(event, callback);
    }

    public static once(id: string, event: string, callback: Callback): void {
        const module = Module.get(id);

        if(module) module.once(event, callback);
    }

    public static off(id: string, event: string, callback: Callback): void {
        const module = Module.get(id);

        if(module) module.off(event, callback);
    }
}

export abstract class Module extends EventBus {
    protected id: string;
    //protected elements: any;

    public constructor(id: string) {
        super();
        this.setProperties(id);
        this.setDependencies();
    }

    private setProperties(id: string): void {
        this.id = id;
        //this.elements = {};
    }

    protected abstract setDependencies(): void;

    /*public addElement(element: UIElement): void {
        const id = element.getAttribute("id");

        this.elements[id] = element;
    }

    public getElement(id: string): UIElement {
        return this.elements[id];
    }*/

    public load(): void {
        Module.add(this);
        this.emit("load");
    }

    public unload(): void {
        Module.remove(this);
        this.emit("unload");
    }

    protected static list: any = {};

    protected static add(module: Module): void {
        Module.list[module.id] = module;
    }

    protected static remove(module: Module): void {
        delete Module.list[module.id];
    }

    public static get(id: string): Module {
        return Module.list[id];
    }
}

export abstract class UIModule extends Module {
    //protected rendered: boolean;

    protected constructor(id: string/*, rendered: boolean = false*/) {
        super(id);
        //this.rendered = rendered;
        this.setHTMLElements();
        this.addEventListeners();
    }

    //protected abstract async addHTMLElements(): Promise<void>;

    protected abstract setHTMLElements(): void;

    protected abstract addEventListeners(): void;

    /*private async getView(view: string): Promise<string> {
    }

    /*public async renderView(view: string, values: any): Promise<string> {
    }

    private getCSSPath(css: string): string {
        return "modules/se.rwth." + this.id + "/css/" + css + ".css";
    }

    private getViewPath(view: string): string {
        return "modules/se.rwth." + this.id + "/views/" + view + ".html";
    }*/
}
