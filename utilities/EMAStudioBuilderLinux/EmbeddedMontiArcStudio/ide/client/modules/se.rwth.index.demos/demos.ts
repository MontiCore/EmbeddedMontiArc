/* (c) https://github.com/MontiCore/monticore */
import {Module} from "../se.rwth.common.module/module";

export class Demo {
    protected $title: JQuery;
    protected $item: JQuery;

    protected index: number;

    public constructor(title: string) {
        this.addHTMLElements();
        this.setTitle(title);
    }

    protected addHTMLElements(): void {

    }

    protected setTitle(title: string): void {
        this.$title.html(title);
    }

    public getTitle(): string {
        return this.$title.html();
    }

    public getIndex(): number {
        return this.$item.index();
    }
}

export class Demos extends Module {
    private static readonly USERNAME: string = "EmbeddedMontiArc";
    private static readonly REPONAME: string = "Demos";
    private static readonly BRANCHNAME: string = "master";


    protected $panel: JQuery;
    protected $list: JQuery;

    protected demos: Demo[];

    protected constructor() {
        super();
        this.$panel = jQuery("#demos-panel");
        this.$list = jQuery("#demos-list");
    }

    protected static instance: Demos;

    public static async create(): Promise<Demos> {

    }


    public show(): void {
        this.$panel.show();
        this.emit("show");
    }

    public hide(): void {
        this.$panel.hide();
        this.emit("hide");
    }


    private async load(): Promise<void> {
        const github = new GitHub();
    }

    private addListItems(contents: Object[]): void {

    }

    private addListItem(content: Object): void {

    }
}
