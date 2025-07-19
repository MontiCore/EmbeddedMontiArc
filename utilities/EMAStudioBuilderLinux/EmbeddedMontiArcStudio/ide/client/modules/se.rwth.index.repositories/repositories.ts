/* (c) https://github.com/MontiCore/monticore */
import {Module} from "../se.rwth.common.framework/module";

export class Repository {

}

export class Repositories extends Module {
    protected $panel: JQuery;
    protected $list: JQuery;

    protected constructor() {
        super();
        this.setHTMLElements();
        this.addEventListeners();
    }

    protected setHTMLElements(): void {

    }

    protected addEventListeners(): void {

    }

    protected static instance: Repositories;

    public static async create(): Promise<Repositories> {

    }


    public show(): void {
        this.$panel.show();
        this.emit("show");
    }

    public hide(): void {
        this.$panel.hide();
        this.emit("hide");
    }


    protected onItemClick(): void {

    }

    protected onBackClick(): void {

    }
}
