/* (c) https://github.com/MontiCore/monticore */
import {Module} from "../se.rwth.common.framework/module";

export class Search extends Module {
    protected $inputQuery: JQuery;
    protected $buttonSearch: JQuery;
    protected $buttonCancel: JQuery;

    protected constructor() {
        super();
        this.setHTMLElements();
        this.addEventListeners();
    }

    protected setHTMLElements(): void {
        this.$inputQuery = jQuery("#search-query");
        this.$buttonSearch = jQuery("#search-search");
        this.$buttonCancel = jQuery("#search-cancel");
    }

    protected addEventListeners(): void {

    }


    public getQuery(): string {
        return this.$inputQuery.val() as string;
    }


    protected onSearchClick(): void {

    }

    protected onKeyUp(): void {

    }

    protected onCancelClick(): void {

    }
}
