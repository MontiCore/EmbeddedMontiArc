"use strict";
var __extends = (this && this.__extends) || (function () {
    var extendStatics = Object.setPrototypeOf ||
        ({ __proto__: [] } instanceof Array && function (d, b) { d.__proto__ = b; }) ||
        function (d, b) { for (var p in b) if (b.hasOwnProperty(p)) d[p] = b[p]; };
    return function (d, b) {
        extendStatics(d, b);
        function __() { this.constructor = d; }
        d.prototype = b === null ? Object.create(b) : (__.prototype = b.prototype, new __());
    };
})();
Object.defineProperty(exports, "__esModule", { value: true });
var module_1 = require("../se.rwth.common.framework/module");
var Search = /** @class */ (function (_super) {
    __extends(Search, _super);
    function Search() {
        var _this = _super.call(this) || this;
        _this.setHTMLElements();
        _this.addEventListeners();
        return _this;
    }
    Search.prototype.setHTMLElements = function () {
        this.$inputQuery = jQuery("#search-query");
        this.$buttonSearch = jQuery("#search-search");
        this.$buttonCancel = jQuery("#search-cancel");
    };
    Search.prototype.addEventListeners = function () {
    };
    Search.prototype.getQuery = function () {
        return this.$inputQuery.val();
    };
    Search.prototype.onSearchClick = function () {
    };
    Search.prototype.onKeyUp = function () {
    };
    Search.prototype.onCancelClick = function () {
    };
    return Search;
}(module_1.Module));
exports.Search = Search;
