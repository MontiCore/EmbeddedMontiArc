webpackJsonp([2],{

/***/ "../../../../../src async recursive":
/***/ (function(module, exports, __webpack_require__) {

var map = {
	"app/cypher/cypher.module": [
		"../../../../../src/app/cypher/cypher.module.ts"
	],
	"app/graph/graph.module": [
		"../../../../../src/app/graph/graph.module.ts"
	],
	"app/metric/metric.module": [
		"../../../../../src/app/metric/metric.module.ts"
	],
	"app/modules/modules.module": [
		"../../../../../src/app/modules/modules.module.ts",
		0
	],
	"app/textual/textual.module": [
		"../../../../../src/app/textual/textual.module.ts"
	]
};
function webpackAsyncContext(req) {
	var ids = map[req];
	if(!ids)
		return Promise.reject(new Error("Cannot find module '" + req + "'."));
	return Promise.all(ids.slice(1).map(__webpack_require__.e)).then(function() {
		return __webpack_require__(ids[0]);
	});
};
webpackAsyncContext.keys = function webpackAsyncContextKeys() {
	return Object.keys(map);
};
module.exports = webpackAsyncContext;
webpackAsyncContext.id = "../../../../../src async recursive";

/***/ }),

/***/ "../../../../../src/app/all-projects.service.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__helper_project__ = __webpack_require__("../../../../../src/app/helper/project.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2_rxjs_Rx__ = __webpack_require__("../../../../rxjs/Rx.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2_rxjs_Rx___default = __webpack_require__.n(__WEBPACK_IMPORTED_MODULE_2_rxjs_Rx__);
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__angular_http__ = __webpack_require__("../../../http/@angular/http.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__helper_httpService__ = __webpack_require__("../../../../../src/app/helper/httpService.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5__helper_globalProperties__ = __webpack_require__("../../../../../src/app/helper/globalProperties.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return AllProjectsService; });
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
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};






var AllProjectsService = AllProjectsService_1 = (function (_super) {
    __extends(AllProjectsService, _super);
    function AllProjectsService(http) {
        var _this = _super.call(this) || this;
        _this.http = http;
        _this.globalPropertyUrl = 'assets/globalProperties.json';
        return _this;
    }
    AllProjectsService.extractAllProjectData = function (res) {
        var json = AllProjectsService_1.extractData(res);
        var result = [];
        for (var _i = 0, json_1 = json; _i < json_1.length; _i++) {
            var jsonObject = json_1[_i];
            var od = new __WEBPACK_IMPORTED_MODULE_1__helper_project__["a" /* Project */](jsonObject['name'], jsonObject['rootPath']);
            result.push(od);
        }
        return result;
    };
    AllProjectsService.prototype.getProjects = function () {
        return this.http.post('getProjects.php', 'assets/')
            .map(AllProjectsService_1.extractAllProjectData)
            .catch(AllProjectsService_1.handleError);
    };
    AllProjectsService.prototype.getGlobalProperties = function () {
        var _this = this;
        return new __WEBPACK_IMPORTED_MODULE_2_rxjs_Rx__["Observable"](function (observer) {
            _this.loadGlobalProperties().subscribe(function (prop) {
                _this.resolveGlobalProperties(prop).then(function (resProp) {
                    observer.next(resProp);
                    observer.complete();
                });
            });
        });
    };
    AllProjectsService.prototype.loadGlobalProperties = function () {
        return this.http.get(this.globalPropertyUrl)
            .map(function (res) {
            var json = AllProjectsService_1.extractData(res);
            return __WEBPACK_IMPORTED_MODULE_5__helper_globalProperties__["a" /* GlobalProperties */].convertJSONToObject(json);
        })
            .catch(AllProjectsService_1.handleErrorWith404Acceptance);
    };
    AllProjectsService.prototype.resolveGlobalProperties = function (prop) {
        var _this = this;
        return new Promise(function (resolve) {
            if (prop == null || prop.defaultProject === '') {
                _this.getProjects().subscribe(function (list) {
                    if (list.length > 0) {
                        resolve(new __WEBPACK_IMPORTED_MODULE_5__helper_globalProperties__["a" /* GlobalProperties */](list[0].name, 'neo4j'));
                    }
                });
            }
            else {
                resolve(prop);
            }
        });
    };
    return AllProjectsService;
}(__WEBPACK_IMPORTED_MODULE_4__helper_httpService__["a" /* HttpService */]));
AllProjectsService = AllProjectsService_1 = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Injectable"])(),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_3__angular_http__["d" /* Http */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_3__angular_http__["d" /* Http */]) === "function" && _a || Object])
], AllProjectsService);

var AllProjectsService_1, _a;
//# sourceMappingURL=all-projects.service.js.map

/***/ }),

/***/ "../../../../../src/app/app.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, "", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/app.component.html":
/***/ (function(module, exports) {

module.exports = "<app-navigation></app-navigation>\n\n<!--\n<div [(ngModel)]=\"colorValue\" ngbRadioGroup>\n  <label class=\"btn btn-primary\">\n    <input type=\"radio\" [value]=\"1\"> Accent (8 Colors)\n  </label>\n  <label class=\"btn btn-primary\">\n    <input type=\"radio\" [value]=\"2\"> Dark 2 (8 Colors)\n  </label>\n  <label class=\"btn btn-primary\">\n    <input type=\"radio\" [value]=\"3\"> Pastel 1 (9 Colors)\n  </label>\n  <label class=\"btn btn-primary\">\n    <input type=\"radio\" [value]=\"4\"> Pastel 2 (8 Colors)\n  </label>\n  <label class=\"btn btn-primary\">\n    <input type=\"radio\" [value]=\"5\"> Set 1 (9 Colors)\n  </label>\n  <label class=\"btn btn-primary\">\n    <input type=\"radio\" [value]=\"6\"> Set 2 (8 Colors)\n  </label>\n  <label class=\"btn btn-primary\">\n    <input type=\"radio\" [value]=\"7\"> Set 3 (12 Colors)\n  </label>\n</div>\n\nNumber of Colors:\n<input id=\"colors\"\n       type=\"number\"\n       min=\"3\"\n       max=\"12\"\n       [ngModel]=\"colorCount\"\n       (ngModelChange)=\"colorCountChanged($event)\">\n\n\n<hr>\n-->\n\n<router-outlet></router-outlet>\n"

/***/ }),

/***/ "../../../../../src/app/app.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return AppComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};

var AppComponent = (function () {
    function AppComponent() {
    }
    return AppComponent;
}());
AppComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-root',
        template: __webpack_require__("../../../../../src/app/app.component.html"),
        styles: [__webpack_require__("../../../../../src/app/app.component.css")]
    })
], AppComponent);

//# sourceMappingURL=app.component.js.map

/***/ }),

/***/ "../../../../../src/app/app.module.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_platform_browser__ = __webpack_require__("../../../platform-browser/@angular/platform-browser.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__angular_platform_browser_animations__ = __webpack_require__("../../../platform-browser/@angular/platform-browser/animations.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__angular_forms__ = __webpack_require__("../../../forms/@angular/forms.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__angular_http__ = __webpack_require__("../../../http/@angular/http.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5__ng_bootstrap_ng_bootstrap__ = __webpack_require__("../../../../@ng-bootstrap/ng-bootstrap/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_6_angular_font_awesome_angular_font_awesome__ = __webpack_require__("../../../../angular-font-awesome/angular-font-awesome.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_7__app_component__ = __webpack_require__("../../../../../src/app/app.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_8__textual_textual_module__ = __webpack_require__("../../../../../src/app/textual/textual.module.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_9__graph_graph_module__ = __webpack_require__("../../../../../src/app/graph/graph.module.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_10__metric_metric_module__ = __webpack_require__("../../../../../src/app/metric/metric.module.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_11__navigation_navigation_module__ = __webpack_require__("../../../../../src/app/navigation/navigation.module.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_12__cypher_cypher_module__ = __webpack_require__("../../../../../src/app/cypher/cypher.module.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return AppModule; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};








/* Feature Modules */



//import { ModulesModule } from './modules/modules.module';
/* Routing Module */


var AppModule = (function () {
    function AppModule() {
    }
    return AppModule;
}());
AppModule = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_2__angular_core__["NgModule"])({
        declarations: [
            __WEBPACK_IMPORTED_MODULE_7__app_component__["a" /* AppComponent */]
        ],
        imports: [
            __WEBPACK_IMPORTED_MODULE_0__angular_platform_browser__["BrowserModule"],
            __WEBPACK_IMPORTED_MODULE_3__angular_forms__["a" /* FormsModule */],
            __WEBPACK_IMPORTED_MODULE_4__angular_http__["a" /* HttpModule */],
            __WEBPACK_IMPORTED_MODULE_1__angular_platform_browser_animations__["a" /* BrowserAnimationsModule */],
            __WEBPACK_IMPORTED_MODULE_5__ng_bootstrap_ng_bootstrap__["a" /* NgbModule */].forRoot(),
            __WEBPACK_IMPORTED_MODULE_6_angular_font_awesome_angular_font_awesome__["a" /* AngularFontAwesomeModule */],
            __WEBPACK_IMPORTED_MODULE_11__navigation_navigation_module__["a" /* NavigationModule */].forRoot(),
            __WEBPACK_IMPORTED_MODULE_8__textual_textual_module__["TextualModule"].forRoot(),
            __WEBPACK_IMPORTED_MODULE_9__graph_graph_module__["GraphModule"].forRoot(),
            __WEBPACK_IMPORTED_MODULE_10__metric_metric_module__["MetricModule"].forRoot(),
            __WEBPACK_IMPORTED_MODULE_12__cypher_cypher_module__["CypherModule"].forRoot()
            //ModulesModule.forRoot()
        ],
        providers: [],
        bootstrap: [__WEBPACK_IMPORTED_MODULE_7__app_component__["a" /* AppComponent */]]
    })
], AppModule);

//# sourceMappingURL=app.module.js.map

/***/ }),

/***/ "../../../../../src/app/color.service.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1_d3_ng2_service__ = __webpack_require__("../../../../d3-ng2-service/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2_colorbrewer__ = __webpack_require__("../../../../colorbrewer/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2_colorbrewer___default = __webpack_require__.n(__WEBPACK_IMPORTED_MODULE_2_colorbrewer__);
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3_rxjs_Rx__ = __webpack_require__("../../../../rxjs/Rx.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3_rxjs_Rx___default = __webpack_require__.n(__WEBPACK_IMPORTED_MODULE_3_rxjs_Rx__);
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return ColorService; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};




var ColorService = (function () {
    function ColorService(d3Service) {
        this._domain = new __WEBPACK_IMPORTED_MODULE_3_rxjs_Rx__["BehaviorSubject"]([]);
        this.domain$ = this._domain.asObservable();
        this.d3 = d3Service.getD3();
        var domain = __WEBPACK_IMPORTED_MODULE_2_colorbrewer__["Set3"][12];
        /*
        let d3 = this.d3;
        let color : ScaleOrdinal<number, string> = d3.scaleOrdinal<number, string>(d3.schemeCategory20   );
        //return color;
    
        for (let i = 0; i < 20; i++) {
          domain.push(color(i).toString());
        }
        domain = domain.filter((c,i) => {
          return i % 2 == 1;
        });
        */
        this._domain.next(domain);
    }
    ColorService.prototype.getColorScheme = function () {
        return this._domain.getValue();
    };
    ColorService.prototype.setColorScheme = function (number, colorCount) {
        var domain;
        var color = 8;
        if (colorCount) {
            color = colorCount;
        }
        switch (number) {
            case 1:
                if (color > 8) {
                    color = 8;
                }
                domain = __WEBPACK_IMPORTED_MODULE_2_colorbrewer__["Accent"][color];
                break;
            case 2:
                if (color > 8) {
                    color = 8;
                }
                domain = __WEBPACK_IMPORTED_MODULE_2_colorbrewer__["Dark2"][color];
                break;
            case 3:
                if (color > 9) {
                    color = 9;
                }
                domain = __WEBPACK_IMPORTED_MODULE_2_colorbrewer__["Pastel1"][color];
                break;
            case 4:
                if (color > 8) {
                    color = 8;
                }
                domain = __WEBPACK_IMPORTED_MODULE_2_colorbrewer__["Pastel2"][color];
                break;
            case 5:
                if (color > 9) {
                    color = 9;
                }
                domain = __WEBPACK_IMPORTED_MODULE_2_colorbrewer__["Set1"][color];
                break;
            case 6:
                if (color > 8) {
                    color = 8;
                }
                domain = __WEBPACK_IMPORTED_MODULE_2_colorbrewer__["Set2"][color];
                break;
            case 7:
                if (color > 12) {
                    color = 12;
                }
                domain = __WEBPACK_IMPORTED_MODULE_2_colorbrewer__["Set3"][color];
                break;
        }
        this._domain.next(domain);
    };
    return ColorService;
}());
ColorService = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Injectable"])(),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1_d3_ng2_service__["a" /* D3Service */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1_d3_ng2_service__["a" /* D3Service */]) === "function" && _a || Object])
], ColorService);

var _a;
//# sourceMappingURL=color.service.js.map

/***/ }),

/***/ "../../../../../src/app/cypher/cypher-data.service.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__angular_http__ = __webpack_require__("../../../http/@angular/http.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__helper_httpService__ = __webpack_require__("../../../../../src/app/helper/httpService.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__all_projects_service__ = __webpack_require__("../../../../../src/app/all-projects.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return CypherDataService; });
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
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};




var CypherDataService = CypherDataService_1 = (function (_super) {
    __extends(CypherDataService, _super);
    function CypherDataService(http, allProjectService) {
        var _this = _super.call(this) || this;
        _this.http = http;
        _this.allProjectService = allProjectService;
        var headers = new __WEBPACK_IMPORTED_MODULE_1__angular_http__["b" /* Headers */]();
        headers.append('Authorization', 'Basic ' + btoa("neo4j:test"));
        headers.append('Content-Type', 'application/json');
        _this.defaultOptions = new __WEBPACK_IMPORTED_MODULE_1__angular_http__["c" /* RequestOptions */]({
            headers: headers
        });
        return _this;
    }
    CypherDataService.prototype.executeStatement = function (statement) {
        var _this = this;
        var statements = statement.split(";");
        statements = statements.map(function (s) {
            return s + ';';
        });
        statements.pop();
        var statementList = [];
        statements.forEach(function (st) {
            statementList.push({
                "statement": st,
                "resultDataContents": ["row", "graph"]
            });
        });
        var body = {
            "statements": statementList
        };
        return new Promise(function (resolve) {
            _this.allProjectService.getGlobalProperties().subscribe(function (global) {
                var path = global.neo4jServer + '/db/data/transaction/commit';
                _this.getData(path, body).subscribe(function (res) {
                    resolve(res);
                });
            });
        });
    };
    CypherDataService.prototype.getData = function (path, body) {
        return this.http.post(path, body, this.defaultOptions)
            .map(CypherDataService_1.extractData)
            .catch(CypherDataService_1.handleError);
    };
    CypherDataService.prototype.getGraph = function () {
        var _this = this;
        return new Promise(function (resolve) {
            _this.executeStatement('MATCH (n) RETURN n;').then(function (nodes) {
                _this.executeStatement('MATCH p=()-[r]->() RETURN p;').then(function (relationships) {
                    var realNodes = nodes.results[0].data.map(function (node) {
                        return node.graph.nodes[0];
                    });
                    var realLinks = relationships.results[0].data.map(function (link) {
                        return link.graph.relationships[0];
                    });
                    resolve({ nodes: realNodes, links: realLinks });
                });
            });
        });
    };
    return CypherDataService;
}(__WEBPACK_IMPORTED_MODULE_2__helper_httpService__["a" /* HttpService */]));
CypherDataService = CypherDataService_1 = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Injectable"])(),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__angular_http__["d" /* Http */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__angular_http__["d" /* Http */]) === "function" && _a || Object, typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_3__all_projects_service__["a" /* AllProjectsService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_3__all_projects_service__["a" /* AllProjectsService */]) === "function" && _b || Object])
], CypherDataService);

var CypherDataService_1, _a, _b;
//# sourceMappingURL=cypher-data.service.js.map

/***/ }),

/***/ "../../../../../src/app/cypher/cypher-routing.module.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__angular_router__ = __webpack_require__("../../../router/@angular/router.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__cypher_cypher_component__ = __webpack_require__("../../../../../src/app/cypher/cypher/cypher.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__cypher_data_service__ = __webpack_require__("../../../../../src/app/cypher/cypher-data.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return CypherRoutingModule; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};




var routes = [
    { path: '', component: __WEBPACK_IMPORTED_MODULE_2__cypher_cypher_component__["a" /* CypherComponent */] }
];
var CypherRoutingModule = (function () {
    function CypherRoutingModule() {
    }
    return CypherRoutingModule;
}());
CypherRoutingModule = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["NgModule"])({
        imports: [__WEBPACK_IMPORTED_MODULE_1__angular_router__["a" /* RouterModule */].forChild(routes)],
        providers: [__WEBPACK_IMPORTED_MODULE_3__cypher_data_service__["a" /* CypherDataService */]],
        exports: [__WEBPACK_IMPORTED_MODULE_1__angular_router__["a" /* RouterModule */]]
    })
], CypherRoutingModule);

//# sourceMappingURL=cypher-routing.module.js.map

/***/ }),

/***/ "../../../../../src/app/cypher/cypher.module.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
Object.defineProperty(__webpack_exports__, "__esModule", { value: true });
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__angular_common__ = __webpack_require__("../../../common/@angular/common.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__cypher_cypher_component__ = __webpack_require__("../../../../../src/app/cypher/cypher/cypher.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__cypher_routing_module__ = __webpack_require__("../../../../../src/app/cypher/cypher-routing.module.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__angular_forms__ = __webpack_require__("../../../forms/@angular/forms.es5.js");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "CypherModule", function() { return CypherModule; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};





var CypherModule = CypherModule_1 = (function () {
    function CypherModule() {
    }
    CypherModule.forRoot = function () {
        return {
            ngModule: CypherModule_1,
            providers: []
        };
    };
    return CypherModule;
}());
CypherModule = CypherModule_1 = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["NgModule"])({
        imports: [
            __WEBPACK_IMPORTED_MODULE_1__angular_common__["CommonModule"],
            __WEBPACK_IMPORTED_MODULE_4__angular_forms__["a" /* FormsModule */],
            __WEBPACK_IMPORTED_MODULE_3__cypher_routing_module__["a" /* CypherRoutingModule */]
        ],
        declarations: [__WEBPACK_IMPORTED_MODULE_2__cypher_cypher_component__["a" /* CypherComponent */]]
    })
], CypherModule);

var CypherModule_1;
//# sourceMappingURL=cypher.module.js.map

/***/ }),

/***/ "../../../../../src/app/cypher/cypher/cypher.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, ".scroll-box {\n  overflow-y: scroll;\n  max-height: 60vh;\n  padding: 1rem;\n  margin-top: 1em;\n}\n", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/cypher/cypher/cypher.component.html":
/***/ (function(module, exports) {

module.exports = "<div class=\"container\">\n  <div class=\"row\">\n      <textarea class=\"form-control\" rows=\"5\" [(ngModel)]='input'></textarea>\n  </div>\n\n  <div class=\"row\">\n  <button class=\"btn btn-secondary btn-sm\" (click)=\"executeStatement()\">Submit</button>\n  </div>\n\n  Query Result:\n  <pre class=\"card card-block scroll-box\"> {{output | json}} </pre>\n\n  Whole Graph: <button class=\"btn btn-secondary btn-sm\" (click)=\"getGraph()\">Update Graph</button>\n  <pre class=\"card card-block scroll-box\"> {{graph | json}} </pre>\n</div>\n"

/***/ }),

/***/ "../../../../../src/app/cypher/cypher/cypher.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__cypher_data_service__ = __webpack_require__("../../../../../src/app/cypher/cypher-data.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return CypherComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};


var CypherComponent = (function () {
    function CypherComponent(cypherDataService) {
        this.cypherDataService = cypherDataService;
        this.input = "MATCH (n) OPTIONAL MATCH (n)-[r]-() RETURN n,r;";
    }
    CypherComponent.prototype.ngOnInit = function () {
        this.executeStatement();
        this.getGraph();
    };
    CypherComponent.prototype.executeStatement = function () {
        var _this = this;
        this.cypherDataService.executeStatement(this.input).then(function (res) {
            _this.output = res;
        });
    };
    CypherComponent.prototype.getGraph = function () {
        var _this = this;
        this.cypherDataService.getGraph().then(function (graph) {
            _this.graph = graph;
        });
    };
    return CypherComponent;
}());
CypherComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-cypher',
        template: __webpack_require__("../../../../../src/app/cypher/cypher/cypher.component.html"),
        styles: [__webpack_require__("../../../../../src/app/cypher/cypher/cypher.component.css")]
    }),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__cypher_data_service__["a" /* CypherDataService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__cypher_data_service__["a" /* CypherDataService */]) === "function" && _a || Object])
], CypherComponent);

var _a;
//# sourceMappingURL=cypher.component.js.map

/***/ }),

/***/ "../../../../../src/app/graph/event-communication.service.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1_rxjs_Rx__ = __webpack_require__("../../../../rxjs/Rx.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1_rxjs_Rx___default = __webpack_require__.n(__WEBPACK_IMPORTED_MODULE_1_rxjs_Rx__);
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return EventCommunicationService; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};


var EventCommunicationService = (function () {
    function EventCommunicationService() {
        this._splitterMoved = new __WEBPACK_IMPORTED_MODULE_1_rxjs_Rx__["Subject"]();
        this._treeItemSelected = new __WEBPACK_IMPORTED_MODULE_1_rxjs_Rx__["Subject"]();
        this._canvasItemsSelected = new __WEBPACK_IMPORTED_MODULE_1_rxjs_Rx__["Subject"]();
        this._searchItemSelected = new __WEBPACK_IMPORTED_MODULE_1_rxjs_Rx__["Subject"]();
        this._itemSelected = new __WEBPACK_IMPORTED_MODULE_1_rxjs_Rx__["Subject"]();
        this._resetZoomClicked = new __WEBPACK_IMPORTED_MODULE_1_rxjs_Rx__["Subject"]();
        this._expandAllClicked = new __WEBPACK_IMPORTED_MODULE_1_rxjs_Rx__["Subject"]();
        this._collapseAllClicked = new __WEBPACK_IMPORTED_MODULE_1_rxjs_Rx__["Subject"]();
        this._typesSelected = new __WEBPACK_IMPORTED_MODULE_1_rxjs_Rx__["Subject"]();
        this.splitterMoved$ = this._splitterMoved.asObservable();
        this.treeItemSelected$ = this._treeItemSelected.asObservable();
        this.canvasItemsSelected$ = this._canvasItemsSelected.asObservable();
        this.searchItemSelected$ = this._searchItemSelected.asObservable();
        this.itemSelected$ = this._itemSelected.asObservable();
        this.resetZoomClicked$ = this._resetZoomClicked.asObservable();
        this.expandAllClicked$ = this._expandAllClicked.asObservable();
        this.collapseAllClicked$ = this._collapseAllClicked.asObservable();
        this.typesSelected$ = this._typesSelected.asObservable();
    }
    EventCommunicationService.prototype.splitterMoved = function () {
        this._splitterMoved.next();
    };
    EventCommunicationService.prototype.selectTreeItem = function (treeItem) {
        this._treeItemSelected.next(treeItem);
        this.itemSelected();
    };
    // Is also called, if tree select is called
    EventCommunicationService.prototype.selectCanvasItems = function (canvasItems) {
        this._canvasItemsSelected.next(canvasItems);
        this.itemSelected();
    };
    EventCommunicationService.prototype.selectSearchItem = function (searchItem) {
        this._searchItemSelected.next(searchItem);
        this.itemSelected();
    };
    EventCommunicationService.prototype.itemSelected = function () {
        this._itemSelected.next();
    };
    EventCommunicationService.prototype.resetZoomClicked = function () {
        this._resetZoomClicked.next();
    };
    EventCommunicationService.prototype.expandAllClicked = function () {
        this._expandAllClicked.next();
    };
    EventCommunicationService.prototype.collapseAllClicked = function () {
        this._collapseAllClicked.next();
    };
    EventCommunicationService.prototype.selectTypes = function (types) {
        this._typesSelected.next(types);
    };
    return EventCommunicationService;
}());
EventCommunicationService = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Injectable"])(),
    __metadata("design:paramtypes", [])
], EventCommunicationService);

//# sourceMappingURL=event-communication.service.js.map

/***/ }),

/***/ "../../../../../src/app/graph/graph-communication.service.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return GraphCommunicationService; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};

var GraphCommunicationService = (function () {
    function GraphCommunicationService() {
        this._selectedNodes = [];
    }
    Object.defineProperty(GraphCommunicationService.prototype, "selectedNodes", {
        get: function () {
            return this._selectedNodes;
        },
        set: function (selected) {
            this._selectedNodes = selected;
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(GraphCommunicationService.prototype, "nodeTypes", {
        get: function () {
            return this._nodeTypes;
        },
        set: function (nodeTypes) {
            this._nodeTypes = nodeTypes;
        },
        enumerable: true,
        configurable: true
    });
    return GraphCommunicationService;
}());
GraphCommunicationService = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Injectable"])(),
    __metadata("design:paramtypes", [])
], GraphCommunicationService);

//# sourceMappingURL=graph-communication.service.js.map

/***/ }),

/***/ "../../../../../src/app/graph/graph-main/graph-canvas/graph-canvas.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, "\n.group text {\n  /*dominant-baseline: text-before-edge;*/\n  cursor: pointer;\n}\n\n.group text:hover {\n  text-decoration: underline;\n}\n\n.link {\n  stroke: black;\n}\n\n.link:hover .linkVisual {\n  stroke: red;\n  marker-end: url(#end-arrow-hover);\n}\n\n.link:hover .dual {\n  marker-start: url(#start-arrow-hover);\n}\n\n.linkVisual {\n  stroke-width:1.5px;\n  opacity: 0.5;\n  marker-end: url(#end-arrow);\n}\n\n.linkInteract {\n  opacity: 0;\n  shape-rendering: crispEdges;\n  stroke-width: 8px;\n  stroke-linecap: round;\n  cursor: pointer;\n}\n\n.dual {\n  marker-start: url(#start-arrow);\n}\n\n.node {\n  cursor: pointer;\n}\n\n.node > rect {\n  stroke: lightgray;\n}\n\n.node:hover > .nodeCaption {\n  text-decoration: underline;\n}\n\n.node > text {\n  dominant-baseline: central;\n  text-anchor: middle;\n  text-rendering: geometricPrecision;\n  font-size : 14px;\n}\n\n.node.selected > rect {\n  stroke: crimson;\n  stroke-width: 1.5px\n}\n\n#start-arrow, #end-arrow {\n  fill: black;\n}\n\n#start-arrow-hover, #end-arrow-hover {\n  fill: red;\n}\n\n* {\n  box-sizing: border-box;\n}\n\nsvg {\n  -webkit-box-flex: 1;\n      -ms-flex: 1;\n          flex: 1;\n  margin: 4px;\n  border: 1px solid darkgray;\n  height: 70.5vh;\n  width: 99%;\n}\n\n#contextMenu {\n  display: none;\n  position: absolute;\n  background: azure;\n  border: 1px solid gray;\n  cursor: move;\n}\n\n#contextMenu span {\n  display: inline-block;\n  padding: 2px;\n  line-height: 1;\n}\n\n#contextMenu > ul {\n  padding: 2px;\n}\n\n#contextMenu > ul ul {\n  padding: 4px 0 4px 4px;\n}\n\n.progress {\n  text-align:left;\n}\n", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/graph/graph-main/graph-canvas/graph-canvas.component.html":
/***/ (function(module, exports) {

module.exports = "<div id=\"graph\">\n  <svg>\n    <defs>\n      <marker id=\"end-arrow\"\n              viewBox=\"0 -5 10 10\"\n              refX=\"8\"\n              markerWidth=\"8\"\n              markerHeight=\"8\"\n              orient=\"auto\">\n        <path d=\"M0,-4L10,0L0,4\" />\n      </marker>\n      <marker id=\"start-arrow\"\n              viewBox=\"-10 -5 10 10\"\n              refX=\"-8\"\n              markerWidth=\"8\"\n              markerHeight=\"8\"\n              orient=\"auto\">\n        <path d=\"M0,-4L-10,0L0,4\" />\n      </marker>\n      <marker id=\"end-arrow-hover\"\n              viewBox=\"0 -5 10 10\"\n              refX=\"8\"\n              markerWidth=\"8\"\n              markerHeight=\"8\"\n              orient=\"auto\">\n        <path d=\"M0,-4L10,0L0,4\" />\n      </marker>\n      <marker id=\"start-arrow-hover\"\n              viewBox=\"-10 -5 10 10\"\n              refX=\"-8\"\n              markerWidth=\"8\"\n              markerHeight=\"8\"\n              orient=\"auto\">\n        <path d=\"M0,-4L-10,0L0,4\" />\n      </marker>\n    </defs>\n  </svg>\n</div>\n<div id=\"contextMenu\"></div>\n<ngb-progressbar type=\"info\" [animated]=\"true\" [value]=status.progressBarValue>{{status.text}}</ngb-progressbar>\n"

/***/ }),

/***/ "../../../../../src/app/graph/graph-main/graph-canvas/graph-canvas.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__ = __webpack_require__("../../../../../src/app/graph/helper/graph-data.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__selected_project_service__ = __webpack_require__("../../../../../src/app/selected-project.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__ = __webpack_require__("../../../../../src/app/graph/helper/graph-canvas-helper.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4_webcola__ = __webpack_require__("../../../../webcola/dist/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4_webcola___default = __webpack_require__.n(__WEBPACK_IMPORTED_MODULE_4_webcola__);
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5_d3_ng2_service__ = __webpack_require__("../../../../d3-ng2-service/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_6__graph_communication_service__ = __webpack_require__("../../../../../src/app/graph/graph-communication.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_7__event_communication_service__ = __webpack_require__("../../../../../src/app/graph/event-communication.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_8__helper_graph_helper_service__ = __webpack_require__("../../../../../src/app/graph/helper/graph-helper.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_9__helper_graph_tree_service__ = __webpack_require__("../../../../../src/app/graph/helper/graph-tree.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return GraphCanvasComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};










var GraphCanvasComponent = (function () {
    function GraphCanvasComponent(d3Service, selectedProjectService, graphCanvasHelperService, eventCommunicationService, graphCommunicationService, graphTreeService) {
        var _this = this;
        this.selectedProjectService = selectedProjectService;
        this.graphCanvasHelperService = graphCanvasHelperService;
        this.eventCommunicationService = eventCommunicationService;
        this.graphCommunicationService = graphCommunicationService;
        this.graphTreeService = graphTreeService;
        this.d3 = d3Service.getD3();
        this.selectedTypes = [];
        // TODO Double Check
        /*selectedProjectService.properties$.subscribe(
          properties => {
            this.groupItemType = properties["graph"].groupItemType;
            this.subGroupItemType = properties["graph"].subGroupItemType;
            this.groupChildLinkTypes = properties["graph"].groupChildLinkTypes;
            this.groupCrossLinkTypes = properties["graph"].groupCrossLinkTypes;
          },
        );*/
        eventCommunicationService.splitterMoved$.subscribe(function () {
            _this.updateViewBox(false);
        });
        eventCommunicationService.treeItemSelected$.subscribe(function (selection) {
            var heldControl = _this.d3.event.ctrlKey;
            _this.selectNodes([selection], _this.clickToAdd || heldControl);
        });
        eventCommunicationService.searchItemSelected$.subscribe(function (selection) {
            _this.selectNodes([selection.item], selection.addToSelection);
        });
        eventCommunicationService.resetZoomClicked$.subscribe(function () {
            _this.resetZoom(true);
        });
        eventCommunicationService.expandAllClicked$.subscribe(function () {
            _this.expandGraph();
        });
        eventCommunicationService.collapseAllClicked$.subscribe(function () {
            _this.collapseGraph();
        });
        eventCommunicationService.typesSelected$.subscribe(function (types) {
            _this.selectedTypes = types;
            if (_this.graph != null) {
                _this.makeGraphWithMessage();
            }
        });
    }
    Object.defineProperty(GraphCanvasComponent.prototype, "graph", {
        get: function () {
            return this._graph;
        },
        set: function (graph) {
            this._graph = graph;
            if (graph != null) {
                this.initView();
            }
        },
        enumerable: true,
        configurable: true
    });
    ;
    Object.defineProperty(GraphCanvasComponent.prototype, "isHierarchical", {
        get: function () {
            return this._isHierarchical;
        },
        set: function (isHierarchical) {
            this._isHierarchical = isHierarchical;
            if (this.graph != null && this.isHierarchical != null) {
                this.updateGraph();
            }
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(GraphCanvasComponent.prototype, "layoutActive", {
        get: function () {
            return this._layoutActive;
        },
        set: function (layoutActive) {
            this._layoutActive = layoutActive;
            if (this.graph != null) {
                this.addDragBehaviour(this.node, this.layout, this.drag, this.layoutActive);
                if (!this.layoutActive) {
                    this.layout.stop();
                }
                else {
                    this.startLayout(this.currentGraph);
                }
            }
        },
        enumerable: true,
        configurable: true
    });
    ;
    Object.defineProperty(GraphCanvasComponent.prototype, "nodeCollision", {
        get: function () {
            return this._nodeCollision;
        },
        set: function (nodeCollision) {
            this._nodeCollision = nodeCollision;
            if (this.graph != null && this.layoutActive) {
                this.makeGraphWithMessage();
            }
        },
        enumerable: true,
        configurable: true
    });
    ;
    Object.defineProperty(GraphCanvasComponent.prototype, "allLinks", {
        get: function () {
            return this._allLinks;
        },
        set: function (allLinks) {
            this._allLinks = allLinks;
            if (this.graph != null) {
                this.makeGraphWithMessage();
            }
        },
        enumerable: true,
        configurable: true
    });
    ;
    Object.defineProperty(GraphCanvasComponent.prototype, "clickToAdd", {
        get: function () {
            return this._clickToAdd;
        },
        set: function (clickToAdd) {
            this._clickToAdd = clickToAdd;
        },
        enumerable: true,
        configurable: true
    });
    ;
    GraphCanvasComponent.prototype.ngOnInit = function () {
        this.initVariables();
    };
    GraphCanvasComponent.prototype.initVariables = function () {
        var _this = this;
        this.svg = this.d3.select('#graph').select('svg');
        this.viewBox = {};
        this.svgGroup = this.svg.append('g');
        this.groupGraphs = {};
        this.group = this.svgGroup.append('g').attr('id', 'groupGroup').selectAll('.group');
        this.path = this.svgGroup.append('g').attr('id', 'linkGroup').selectAll('.link');
        this.node = this.svgGroup.append('g').attr('id', 'nodeGroup').selectAll('.node');
        this.groupMargin = 16;
        this.nodePadding = 2;
        this.nodeMargin = 8;
        this.selectedNodes = [];
        this.fixedNodes = [];
        this.zoom = this.d3.zoom().on('zoom', function () {
            _this.svgGroup.attr('transform', _this.d3.event.transform);
            _this.d3.select('#contextMenu').style('display', null);
        });
        this.svg.call(this.zoom);
        this.svg.style('cursor', 'default');
        this.status = { progressBarValue: 0, text: '' };
    };
    GraphCanvasComponent.prototype.initView = function () {
        var _this = this;
        this.loadProperties().then(function () {
            _this.resetSvg();
            _this.initGraphs();
            _this.setUpLayout();
            // Check MenuButtons
        }, function (error) { return console.log(error); });
    };
    GraphCanvasComponent.prototype.loadProperties = function () {
        var _this = this;
        return new Promise(function (resolve, reject) {
            _this.selectedProjectService.getProperties()
                .subscribe(function (properties) {
                // if (properties.hasOwnProperty('graph')) {
                _this.groupItemType = properties.graph.groupItemType;
                _this.subGroupItemType = properties.graph.subGroupItemType;
                _this.groupChildLinkTypes = properties.graph.groupChildLinkTypes;
                _this.groupCrossLinkTypes = properties.graph.groupCrossLinkTypes;
                /* } else {
                   this.groupItemType = '';
                  this.subGroupItemType = [];
                  this.groupChildLinkTypes = [];
                  this.groupCrossLinkTypes = [];
                } */
                resolve();
            }, function (error) {
                reject(error);
            });
        });
    };
    GraphCanvasComponent.prototype.updateGraph = function () {
        if (this.isHierarchical) {
            this.makeHierarchicalGraph();
        }
        else {
            this.makeGraphWithMessage();
        }
    };
    GraphCanvasComponent.prototype.resetSvg = function () {
        var svg = this.svg;
        if (svg) {
            svg.selectAll('g').remove();
            this.svgGroup = this.svg.append('g');
            this.group = this.svgGroup.append('g').attr('id', 'groupGroup').selectAll('.group');
            this.path = this.svgGroup.append('g').attr('id', 'linkGroup').selectAll('.link');
            this.node = this.svgGroup.append('g').attr('id', 'nodeGroup').selectAll('.node');
        }
    };
    GraphCanvasComponent.prototype.initGraphs = function () {
        this.groupGraphs[this.groupItemType] =
            this.graphCanvasHelperService.createGroupGraphInitial(this.groupItemType, this.groupChildLinkTypes, this.groupCrossLinkTypes, this.graph);
        this.hierarchicalGraph = {
            elements: [].concat(this.groupGraphs[this.groupItemType].elements),
            links: [].concat(this.groupGraphs[this.groupItemType].links),
            groups: [],
            isHierarchical: true
        };
        this._graph.groups = [];
        this.fullGraph = this.graph;
        // Show nothing at beginning
        this.currentGraph = { elements: [], links: [], groups: [] };
    };
    GraphCanvasComponent.prototype.setUpLayout = function () {
        var _this = this;
        var d3 = this.d3;
        this.layout = __WEBPACK_IMPORTED_MODULE_4_webcola__["d3adaptor"](d3);
        this.layout
            .on('tick', function () {
            _this.tick();
        })
            .on('start', function () {
            _this.setProgressBar(0, 'layouting...');
        })
            .on('end', function () {
            _this.fixNodes([]);
            _this.setProgressBar(100, 'Layout finished.');
        });
        this.drag = d3.drag().subject(function (d) {
            return d;
        }).on('start', function (d, index, elements) {
            // close context Menu
            var context = d3.select('#contextMenu');
            context.style('display', null);
            // don't propagate to zoom behaviour
            // d3.event.sourceEvent.stopPropagation();
            d3.select(elements[index]).classed('dragging', true);
        }).on('drag', function (d, index, elements) {
            _this.fixNodes([]);
            d['x'] = d3.event.x;
            d['y'] = d3.event.y;
            _this.tick();
            d3.select(elements[index])
                .attr('transform', 'translate(' + d['x'] + ',' + d['y'] + ')');
        }).on('end', function (d, index, elements) {
            d3.select(elements[index]).classed('dragging', false);
        });
        d3.select('#contextMenu').call(d3.drag().on('drag', function () {
            var cm = d3.select(this);
            var left = Number.parseInt(cm.style('left'));
            var top = Number.parseInt(cm.style('top'));
            cm.style('left', left + d3.event.dx + 'px');
            cm.style('top', top + d3.event.dy + 'px');
        }));
        // context menu closing
        ['click', 'wheel'].forEach(function (type) {
            d3.select('body').on(type, function () {
                d3.select('#contextMenu').style('display', null);
            });
        });
        this.updateViewBox(true);
    };
    GraphCanvasComponent.prototype.fixNodes = function (nodes) {
        this.fixedNodes.forEach(function (n) {
            n.fixed = false;
        });
        nodes.forEach(function (n) {
            n.fixed = true;
        });
        this.fixedNodes = nodes;
    };
    GraphCanvasComponent.prototype.tick = function () {
        var _this = this;
        var d3 = this.d3;
        var groups = d3.selectAll('#groupGroup').selectAll('g');
        var paths = d3.select('#linkGroup').selectAll('g');
        var nodes = d3.select('#nodeGroup').selectAll('g');
        // avoid node overlapping
        this.layout.avoidOverlaps(this.layoutActive);
        groups.attr('transform', function (d) {
            return 'translate(' + d['bounds'].x + ',' + d['bounds'].y + ')';
        });
        groups.selectAll('rect').attr('width', function (d) {
            return d['bounds'].X - d['bounds'].x - 32;
        }).attr('height', function (d) {
            return d['bounds'].Y - d['bounds'].y - 32;
        });
        //  draw links at node's border
        paths.selectAll('path').attr('d', function (d) {
            var targetPos = _this.calcArrowPosition(d['source'], d['target']);
            var sourcePos = _this.calcArrowPosition(d['target'], d['source']);
            return 'M' + sourcePos.x + ',' + sourcePos.y +
                'L' + targetPos.x + ',' + targetPos.y;
        });
        nodes.attr('transform', function (d) {
            return 'translate(' + d['x'] + ',' + d['y'] + ')';
        });
    };
    GraphCanvasComponent.prototype.calcArrowPosition = function (source, target) {
        var pad = 2 - this.nodeMargin, dx = target.x - source.x, dy = target.y - source.y, hw = target.width * 0.5 + pad, hh = target.height * 0.5 + pad, nx = dx < 0 ? -hh : hh, ny = dy < 0 ? hw : -hw, prod = dx * nx + dy * ny, bx = (dx < 0 ? hw : -hw), by = (dy < 0 ? hh : -hh), targetX = target.x + (prod > 0 ? bx : by * dx / dy), targetY = target.y + (prod > 0 ? bx * dy / dx : by);
        return { x: targetX, y: targetY };
    };
    GraphCanvasComponent.prototype.updateViewBox = function (reset) {
        var _this = this;
        var bBox = function () {
            return _this.svg.node().getBoundingClientRect();
        };
        if (reset) {
            this.viewBox.x = 0;
            this.viewBox.y = 0;
        }
        else {
            this.viewBox.x += (this.viewBox.w - bBox().width) * 0.5;
            this.viewBox.y += (this.viewBox.h - bBox().height) * 0.5;
        }
        this.viewBox.w = bBox().width;
        this.viewBox.h = bBox().height;
        this.svg.attr('viewBox', this.viewBox.x + ' ' + this.viewBox.y + ' ' + this.viewBox.w + ' ' + this.viewBox.h);
        this.layout.size([
            this.viewBox.w,
            this.viewBox.h
        ]);
    };
    GraphCanvasComponent.prototype.selectNodes = function (nodes, addOrRemove) {
        var _this = this;
        if (addOrRemove) {
            var unselectedNodes = nodes.filter(function (n) {
                return _this.selectedNodes.indexOf(n) < 0;
            });
            if (unselectedNodes.length) {
                unselectedNodes.forEach(function (n) {
                    _this.selectedNodes.push(n);
                });
            }
            else {
                nodes.forEach(function (n) {
                    // remove node from selection
                    _this.selectedNodes.splice(_this.selectedNodes.indexOf(n), 1);
                });
            }
        }
        else {
            this.resetZoom(false);
            this.selectedNodes = nodes;
        }
        this.eventCommunicationService.selectCanvasItems(this.selectedNodes);
        this.makeGraphWithMessage();
    };
    GraphCanvasComponent.prototype.resetZoom = function (resetScale) {
        var d3 = this.d3;
        var zoom = this.zoom;
        var transformation = d3.zoomIdentity;
        if (!resetScale) {
            // center view using current scale
            var currentTransform = d3.zoomTransform(this.svg.node());
            var dx = -this.viewBox.w * (currentTransform.k - 1) * 0.5;
            var dy = -this.viewBox.h * (currentTransform.k - 1) * 0.5;
            transformation = transformation.translate(dx, dy);
        }
        this.svgGroup.call(zoom.transform, transformation);
        this.svg.transition()
            .duration(750)
            .call(zoom.transform, d3.zoomIdentity);
    };
    GraphCanvasComponent.prototype.makeGraphWithMessage = function () {
        this.setProgressBar(0, 'Making graph...');
        this.makeGraph();
    };
    /*
     make and show flat Graph
     */
    GraphCanvasComponent.prototype.makeGraph = function () {
        // exclude unselected types
        var selectedTypes = this.selectedTypes;
        var filteredNodes = this.fullGraph.elements.filter(function (e) {
            return selectedTypes.indexOf(e.type) > -1;
        });
        var filteredLinks = this.fullGraph.links.filter(function (l) {
            return filteredNodes.indexOf(l.source) > -1 && filteredNodes.indexOf(l.target) > -1;
        });
        var graph = { elements: filteredNodes, links: filteredLinks };
        // show only selected filtered nodes and their neighbours
        graph = __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].closedNeighbourhood(graph, this.selectedNodes.filter(function (n) {
            return selectedTypes.indexOf(n.type) > -1;
        }), this.allLinks);
        this.currentGraph = graph;
        this.makeGraphElements();
    };
    GraphCanvasComponent.prototype.makeGraphElements = function () {
        var _this = this;
        var graph = this.currentGraph;
        Promise.all([
            this.makeNodeElements(graph),
            this.makeGroupElements(graph),
            this.makePathElements(graph)
        ]).then(function (responses) {
            _this.setProgressBar(90);
            var nodeGroup = responses[0];
            _this.layout.avoidOverlaps(_this.nodeCollision || graph.groups.length);
            _this.layout.nodes(graph.elements);
            _this.layout.links(graph.links);
            if (graph.groups && graph.groups.length) {
                _this.layout.groups(graph.groups);
            }
            else {
                _this.layout.groups([]);
            }
            _this.setProgressBar(100);
            _this.startLayout(graph);
            _this.addDragBehaviour(nodeGroup, _this.layout, _this.drag, _this.layoutActive);
        });
    };
    GraphCanvasComponent.prototype.makeNodeElements = function (graph) {
        var _this = this;
        return new Promise(function (resolve) {
            var d3 = _this.d3;
            _this.node = d3.select('#nodeGroup').selectAll('g').data(graph.elements, function (d) {
                return __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].getNodeId(d);
            });
            _this.node.exit().remove();
            var nodeGroup = _this.node.enter().append('g')
                .classed('node', true)
                .on('mousedown.zoom', function () {
                // allow dragging of nodes without panning the whole graph
                d3.event.stopPropagation();
            });
            _this.node = nodeGroup.merge(_this.node);
            var text = nodeGroup.append('text').classed('nodeCaption', true)
                .text(function (d) {
                if (d.isGroup && graph.isHierarchical) {
                    return __WEBPACK_IMPORTED_MODULE_8__helper_graph_helper_service__["a" /* GraphHelperService */].getNodeName(d) + ' [+]';
                }
                return __WEBPACK_IMPORTED_MODULE_8__helper_graph_helper_service__["a" /* GraphHelperService */].getNodeName(d);
            });
            graph.elements.forEach(function (e, i) {
                if (!e.width) {
                    e.width = text._groups[0][i].getBBox().width + 2 * _this.nodePadding + 2 * _this.nodeMargin;
                    e.height = text._groups[0][i].getBBox().height + 2 * _this.nodePadding + 2 * _this.nodeMargin;
                }
            });
            var rect = nodeGroup.insert('rect', 'text')
                .style('fill', function (d) {
                return _this.graphCommunicationService.nodeTypes[d.type];
            })
                .attr('x', function (d) {
                return -d.width * 0.5 + _this.nodeMargin;
            })
                .attr('y', function (d) {
                return -d.height * 0.5 + _this.nodeMargin;
            })
                .attr('width', function (d) {
                return d.width - 2 * _this.nodeMargin;
            })
                .attr('height', function (d) {
                return d.height - 2 * _this.nodeMargin;
            });
            d3.selectAll('.selected').classed('selected', false);
            d3.selectAll('.node').filter(function (n) {
                return _this.selectedNodes.indexOf(n) > -1;
            }).classed('selected', true);
            d3.selectAll('.treeNodeCaption').filter(function (n) {
                return _this.selectedNodes.indexOf(n) > -1;
            }).classed('selected', true);
            nodeGroup.on('click', function (d) {
                // ignore if dragging
                if (!d3.event.defaultPrevented) {
                    if (d.isGroup && graph.isHierarchical) {
                        _this.openGroup(graph, d);
                    }
                    else {
                        if (graph.isHierarchical) {
                            d = _this.fullGraph.elements.find(function (e) {
                                return __WEBPACK_IMPORTED_MODULE_8__helper_graph_helper_service__["a" /* GraphHelperService */].elementEquals(d, e);
                            });
                        }
                        var clickToAdd = _this.clickToAdd ?
                            !d3.event.ctrlKey : d3.event.ctrlKey;
                        if (clickToAdd && _this.selectedNodes.indexOf(d) < 0) {
                            _this.fixNodes([d]);
                        }
                        _this.selectNodes([d], clickToAdd);
                    }
                }
            });
            nodeGroup.on('contextmenu', function (d) {
                var contextMenu = d3.select('#contextMenu').style('display', 'block');
                var menu = __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].createContextMenu(contextMenu);
                var li = menu.append('ul').append('li');
                li.append('span').style('background', _this.graphCommunicationService.nodeTypes[d.type]).text(__WEBPACK_IMPORTED_MODULE_8__helper_graph_helper_service__["a" /* GraphHelperService */].getNodeName(d));
                li.append('span').text(' : ' + d.type + ' {');
                __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].appendProperties(li, d.attributes);
                li.append('span').text('}');
                __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].showContextMenu(menu, d3.event, d3.mouse(document.body));
            });
            resolve(nodeGroup);
        });
    };
    GraphCanvasComponent.prototype.makeGroupElements = function (graph) {
        var _this = this;
        return new Promise(function (resolve) {
            var d3 = _this.d3;
            _this.group = d3.select('#groupGroup').selectAll('g').data(graph.groups, function (d) {
                return 'group' + d.id;
            });
            _this.group.exit().remove();
            var groupGroup = _this.group.enter().append('g').classed('group', true);
            _this.group = groupGroup.merge(_this.group);
            groupGroup.append('rect')
                .attr('x', _this.groupMargin).attr('y', _this.groupMargin)
                .style('stroke', 'black')
                .style('fill', function (d) {
                return _this.graphCommunicationService.nodeTypes[d.type];
            });
            // let groupCaption = groupGroup.append('text').html(function(d) {
            groupGroup.append('text').html(function (d) {
                return d.name + ' [&minus;]';
            }).on('click', function (d) {
                _this.closeGroup(graph, d);
            }).style('cursor', 'pointer')
                .attr('x', _this.groupMargin + _this.nodePadding).attr('y', _this.groupMargin + _this.nodePadding + 10);
            resolve();
        });
    };
    GraphCanvasComponent.prototype.makePathElements = function (graph) {
        var _this = this;
        return new Promise(function (resolve) {
            var d3 = _this.d3;
            _this.path = d3.select('#linkGroup').selectAll('g').data(graph.links, function (d) {
                return __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].getNodeId(d['source']) + '>' + __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].getNodeId(d['target']);
            });
            _this.path.exit().remove();
            var linkGroup = _this.path.enter().append('g').classed('link', true);
            _this.path = linkGroup.merge(_this.path);
            linkGroup.append('svg:path')
                .attr('class', function (d) {
                if (__WEBPACK_IMPORTED_MODULE_8__helper_graph_helper_service__["a" /* GraphHelperService */].isEmpty(d.backwardLinks)) {
                    return 'linkVisual';
                }
                else {
                    return 'linkVisual dual';
                }
            });
            linkGroup.append('svg:path').attr('class', 'linkInteract');
            linkGroup.on('click', function (d) {
                var source = d.source;
                var target = d.target;
                if (graph.isHierarchical) {
                    // Dont switch from hierarchy to flat on link press
                }
                else {
                    // ignore if dragging
                    if (!d3.event.defaultPrevented) {
                        if (d3.event.ctrlKey) {
                            _this.fixNodes([source, target].filter(function (n) {
                                //  set previous position for fixing
                                n.px = n.x;
                                n.py = n.y;
                                return _this.selectedNodes.indexOf(n) < 0;
                            }));
                        }
                        _this.selectNodes([source, target], d3.event.ctrlKey);
                    }
                }
            });
            linkGroup.on('contextmenu', function (d) {
                var contextMenu = d3.select('#contextMenu').style('display', 'block');
                var menu = __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].createContextMenu(contextMenu), list = menu.append('ul');
                _this.graphCanvasHelperService.appendLinkDescription(list, d.forwardLinks, d);
                if (!__WEBPACK_IMPORTED_MODULE_8__helper_graph_helper_service__["a" /* GraphHelperService */].isEmpty(d.backwardLinks)) {
                    _this.graphCanvasHelperService.appendLinkDescription(list, d.backwardLinks, d);
                }
                __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].showContextMenu(menu, d3.event, d3.mouse(document.body));
            });
            resolve();
        });
    };
    GraphCanvasComponent.prototype.startLayout = function (graph) {
        var _this = this;
        this.updateViewBox(true);
        var logN = Math.log2(graph.elements.length);
        var preTicks = (graph.elements.length / 100) + (graph.groups.length * 2);
        preTicks = Math.max(preTicks, 5);
        preTicks = Math.min(preTicks, 50);
        if (graph.links.length) {
            this.layout.linkDistance(logN * 64);
        }
        this.setProgressBar(0, 'precalculating layout...');
        this.updateProgressBar(preTicks * 100);
        setTimeout(function () {
            _this.layout.start(preTicks, preTicks, preTicks);
            _this.setProgressBar(0, 'layouting...');
            // stop layout when running too long
            var stopLayout = function () {
                _this.layout.stop();
            };
            var layoutTimeFactor = 500;
            if (graph.groups.length) {
                layoutTimeFactor *= 2;
            }
            var layoutTime = Math.max(logN * layoutTimeFactor, 4000);
            _this.updateProgressBar(layoutTime);
            window.clearTimeout(_this.stopLayoutTimeout);
            _this.stopLayoutTimeout = window.setTimeout(stopLayout, layoutTime);
        }, 0);
    };
    GraphCanvasComponent.prototype.addDragBehaviour = function (nodes, layout, drag, layoutOnDrag) {
        if (layoutOnDrag) {
            nodes.call(layout.drag());
        }
        else {
            nodes.call(drag);
        }
    };
    /*
     make and show hierarchical graph
     */
    GraphCanvasComponent.prototype.makeHierarchicalGraph = function () {
        __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].reOrderGraph(this.hierarchicalGraph);
        __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].reIndexGroups(this.hierarchicalGraph);
        this.currentGraph = this.hierarchicalGraph;
        this.makeGraphElements();
    };
    GraphCanvasComponent.prototype.openGroup = function (graph, group) {
        this.expandGroupIntern(graph, group);
        graph.links = this.graphCanvasHelperService.createGraphLinks(graph.elements, this.groupCrossLinkTypes, this.fullGraph);
        this.graphCanvasHelperService.mapLinks(graph);
        __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].reOrderGraph(graph);
        this.makeGraphElements();
    };
    GraphCanvasComponent.prototype.closeGroup = function (graph, group) {
        this.closeSubGroup(graph, group);
        graph.links = this.graphCanvasHelperService.createGraphLinks(graph.elements, this.groupCrossLinkTypes, this.fullGraph);
        this.graphCanvasHelperService.mapLinks(graph);
        __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].reOrderGraph(graph);
        this.makeGraphElements();
    };
    GraphCanvasComponent.prototype.closeSubGroup = function (graph, group) {
        var _this = this;
        var groupTree = this.graphTreeService.getSubTree(group.id);
        var children = groupTree.children;
        var element = groupTree.element;
        children.forEach(function (ch) {
            var subElement = ch.element;
            if (subElement.isGroup) {
                var subGroup = graph.groups.find(function (g) {
                    return __WEBPACK_IMPORTED_MODULE_8__helper_graph_helper_service__["a" /* GraphHelperService */].elementEquals(g, subElement);
                });
                if (typeof subGroup !== 'undefined') {
                    _this.closeSubGroup(graph, subGroup);
                }
            }
            var elementInGraph = graph.elements.find(function (e) {
                return __WEBPACK_IMPORTED_MODULE_8__helper_graph_helper_service__["a" /* GraphHelperService */].elementEquals(subElement, e);
            });
            graph.elements.splice(graph.elements.indexOf(elementInGraph), 1);
        });
        element.isGroup = true;
        if (groupTree.parent != null) {
            element.myParent = graph.groups.find(function (g) {
                return __WEBPACK_IMPORTED_MODULE_8__helper_graph_helper_service__["a" /* GraphHelperService */].elementEquals(g, groupTree.parent);
            });
        }
        else {
            element.myParent = null;
        }
        graph.elements.push(element);
        var elementInGroups = graph.groups.find(function (e) {
            return element.id === e.id;
        });
        graph.groups.splice(graph.groups.indexOf(elementInGroups), 1);
        var topGroup = group.myParent;
        if (topGroup != null) {
            var elementInTopGroup = topGroup.groups.find(function (e) {
                return element.id === e.id;
            });
            topGroup.groups.splice(topGroup.groups.indexOf(elementInTopGroup), 1);
            topGroup.leaves.push(element);
        }
    };
    GraphCanvasComponent.prototype.expandGroupIntern = function (graph, group) {
        var _this = this;
        __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].restoreLinks(graph);
        //  remove closed-group element
        var index = graph.elements.indexOf(group);
        graph.elements.splice(index, 1);
        __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].reIndexGroups(graph);
        var children = this.graphTreeService.getSubTree(group.id).children;
        var newGroup = {
            id: group.id,
            name: __WEBPACK_IMPORTED_MODULE_8__helper_graph_helper_service__["a" /* GraphHelperService */].getNodeName(group),
            type: group.type,
            myParent: group.myParent,
            padding: 32,
            leaves: [],
            inHierarchyGraph: true
        };
        children.forEach(function (ch) {
            var element = ch.element;
            var isGroupElement = false;
            var filter = false;
            if (typeof _this.subGroupItemType !== 'undefined' && _this.subGroupItemType != null && _this.subGroupItemType.length > 0) {
                for (var _i = 0, _a = _this.subGroupItemType; _i < _a.length; _i++) {
                    var type = _a[_i];
                    if (element.type === type) {
                        filter = true;
                    }
                }
            }
            if (element.type === _this.groupItemType || filter) {
                isGroupElement = true;
            }
            var hasChildren = ch.children.length > 0;
            element.myParent = newGroup;
            element.isGroup = isGroupElement && hasChildren;
            graph.elements.push(element);
            newGroup.leaves.push(element);
        });
        graph.groups.push(newGroup);
        var parentIndex;
        if (newGroup.myParent === null) {
            parentIndex = -1;
            // graph.groups.push(newGroup);
        }
        else {
            var topGroup = newGroup.myParent;
            parentIndex = graph.groups.indexOf(topGroup);
            if (typeof topGroup.groups === 'undefined') {
                topGroup.groups = [];
            }
            topGroup.groups.push(newGroup);
            topGroup.leaves.splice(topGroup.leaves.indexOf(-1), 1);
        }
    };
    GraphCanvasComponent.prototype.expandGraph = function () {
        var graph = this.currentGraph;
        var groups = graph.elements.filter(function (e) {
            return e.isGroup;
        });
        while (groups.length > 0) {
            var group = groups.shift(); // Get first element
            this.expandGroupIntern(graph, group);
            groups = graph.elements.filter(function (e) {
                return e.isGroup;
            });
        }
        graph.links = this.graphCanvasHelperService.createGraphLinks(graph.elements, this.groupCrossLinkTypes, this.fullGraph);
        this.graphCanvasHelperService.mapLinks(graph);
        __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */].reOrderGraph(graph);
        this.currentGraph = graph;
        this.makeGraphElements();
    };
    GraphCanvasComponent.prototype.collapseGraph = function () {
        this.currentGraph = { elements: [], links: [], groups: [] };
        this.makeGraph();
        this.hierarchicalGraph = {
            elements: [].concat(this.groupGraphs[this.groupItemType].elements),
            links: [].concat(this.groupGraphs[this.groupItemType].links),
            groups: [],
            isHierarchical: true
        };
        this.makeHierarchicalGraph();
    };
    GraphCanvasComponent.prototype.updateProgressBar = function (layoutTime) {
        var _this = this;
        var steps = 5;
        var delay = layoutTime / (100 / steps);
        this.status.progressBarValue += steps;
        if (this.status.progressBarValue < 90) {
            setTimeout(function () {
                _this.updateProgressBar(layoutTime);
            }, delay);
        }
    };
    GraphCanvasComponent.prototype.setProgressBar = function (value, text) {
        this.status.progressBarValue = value;
        if (text) {
            this.status.text = text;
        }
    };
    return GraphCanvasComponent;
}());
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */]) === "function" && _a || Object),
    __metadata("design:paramtypes", [typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */]) === "function" && _b || Object])
], GraphCanvasComponent.prototype, "graph", null);
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", Boolean),
    __metadata("design:paramtypes", [Boolean])
], GraphCanvasComponent.prototype, "isHierarchical", null);
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", Boolean),
    __metadata("design:paramtypes", [Boolean])
], GraphCanvasComponent.prototype, "layoutActive", null);
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", Boolean),
    __metadata("design:paramtypes", [Boolean])
], GraphCanvasComponent.prototype, "nodeCollision", null);
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", Boolean),
    __metadata("design:paramtypes", [Boolean])
], GraphCanvasComponent.prototype, "allLinks", null);
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", Boolean),
    __metadata("design:paramtypes", [Boolean])
], GraphCanvasComponent.prototype, "clickToAdd", null);
GraphCanvasComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-graph-canvas',
        template: __webpack_require__("../../../../../src/app/graph/graph-main/graph-canvas/graph-canvas.component.html"),
        styles: [__webpack_require__("../../../../../src/app/graph/graph-main/graph-canvas/graph-canvas.component.css")],
        encapsulation: __WEBPACK_IMPORTED_MODULE_0__angular_core__["ViewEncapsulation"].None
    }),
    __metadata("design:paramtypes", [typeof (_c = typeof __WEBPACK_IMPORTED_MODULE_5_d3_ng2_service__["a" /* D3Service */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_5_d3_ng2_service__["a" /* D3Service */]) === "function" && _c || Object, typeof (_d = typeof __WEBPACK_IMPORTED_MODULE_2__selected_project_service__["a" /* SelectedProjectService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_2__selected_project_service__["a" /* SelectedProjectService */]) === "function" && _d || Object, typeof (_e = typeof __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_3__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */]) === "function" && _e || Object, typeof (_f = typeof __WEBPACK_IMPORTED_MODULE_7__event_communication_service__["a" /* EventCommunicationService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_7__event_communication_service__["a" /* EventCommunicationService */]) === "function" && _f || Object, typeof (_g = typeof __WEBPACK_IMPORTED_MODULE_6__graph_communication_service__["a" /* GraphCommunicationService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_6__graph_communication_service__["a" /* GraphCommunicationService */]) === "function" && _g || Object, typeof (_h = typeof __WEBPACK_IMPORTED_MODULE_9__helper_graph_tree_service__["a" /* GraphTreeService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_9__helper_graph_tree_service__["a" /* GraphTreeService */]) === "function" && _h || Object])
], GraphCanvasComponent);

var _a, _b, _c, _d, _e, _f, _g, _h;
//# sourceMappingURL=graph-canvas.component.js.map

/***/ }),

/***/ "../../../../../src/app/graph/graph-main/graph-legend/graph-legend.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, "#legend {\n  margin: 4px;\n}\n\n#legend label {\n  padding: 4px;\n  display: inline-block;\n  font-size: 14px;\n  font-weight: bold;\n}\n", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/graph/graph-main/graph-legend/graph-legend.component.html":
/***/ (function(module, exports) {

module.exports = "<div id=\"legend\"></div>\n"

/***/ }),

/***/ "../../../../../src/app/graph/graph-main/graph-legend/graph-legend.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1_d3_ng2_service__ = __webpack_require__("../../../../d3-ng2-service/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__graph_communication_service__ = __webpack_require__("../../../../../src/app/graph/graph-communication.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__helper_graph_data__ = __webpack_require__("../../../../../src/app/graph/helper/graph-data.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__event_communication_service__ = __webpack_require__("../../../../../src/app/graph/event-communication.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return GraphLegendComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};





var GraphLegendComponent = (function () {
    function GraphLegendComponent(d3Service, graphCommunicationService, eventCommunicationService) {
        this.graphCommunicationService = graphCommunicationService;
        this.eventCommunicationService = eventCommunicationService;
        this.d3 = d3Service.getD3();
    }
    Object.defineProperty(GraphLegendComponent.prototype, "graph", {
        set: function (graph) {
            if (graph != null) {
                this.resetLegend();
                this.setUpLegend();
            }
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(GraphLegendComponent.prototype, "isHierarchical", {
        set: function (isHierarchical) {
            this.switchHierarchyMode(isHierarchical);
        },
        enumerable: true,
        configurable: true
    });
    GraphLegendComponent.prototype.ngOnInit = function () {
    };
    GraphLegendComponent.prototype.resetLegend = function () {
        if (document.getElementById('legend')) {
            var legend = document.getElementById('legend');
            while (legend.firstChild) {
                legend.removeChild(legend.firstChild);
            }
        }
    };
    GraphLegendComponent.prototype.setUpLegend = function () {
        var _this = this;
        var d3 = this.d3;
        var graphCommunicationService = this.graphCommunicationService;
        this.selectedTypes = Object.keys(graphCommunicationService.nodeTypes);
        this.eventCommunicationService.selectTypes(this.selectedTypes);
        var legend = d3.select('#legend').selectAll('label')
            .data(Object.keys(graphCommunicationService.nodeTypes))
            .enter()
            .append('label')
            .style('background', function (d) {
            return graphCommunicationService.nodeTypes[d];
        });
        legend.append('input')
            .attrs({ 'type': 'checkbox', 'checked': true })
            .on('change', function (e) {
            _this.typeChecked(e);
        });
        legend.append('text').text(function (d) {
            return d;
        });
    };
    GraphLegendComponent.prototype.switchHierarchyMode = function (on) {
        if (on) {
            this.d3.selectAll('#legend input')
                .attr('disabled', 'disabled');
            this.d3.selectAll('#legend input')
                .property('checked', true);
        }
        else {
            this.d3.selectAll('#legend input')
                .attr('disabled', null);
        }
    };
    GraphLegendComponent.prototype.typeChecked = function (type) {
        this.selectedTypes = this.d3.select('#legend').selectAll('input:checked').data();
        this.eventCommunicationService.selectTypes(this.selectedTypes);
    };
    return GraphLegendComponent;
}());
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_3__helper_graph_data__["a" /* GraphData */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_3__helper_graph_data__["a" /* GraphData */]) === "function" && _a || Object),
    __metadata("design:paramtypes", [typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_3__helper_graph_data__["a" /* GraphData */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_3__helper_graph_data__["a" /* GraphData */]) === "function" && _b || Object])
], GraphLegendComponent.prototype, "graph", null);
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", Boolean),
    __metadata("design:paramtypes", [Boolean])
], GraphLegendComponent.prototype, "isHierarchical", null);
GraphLegendComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-graph-legend',
        template: __webpack_require__("../../../../../src/app/graph/graph-main/graph-legend/graph-legend.component.html"),
        styles: [__webpack_require__("../../../../../src/app/graph/graph-main/graph-legend/graph-legend.component.css")],
        encapsulation: __WEBPACK_IMPORTED_MODULE_0__angular_core__["ViewEncapsulation"].None
    }),
    __metadata("design:paramtypes", [typeof (_c = typeof __WEBPACK_IMPORTED_MODULE_1_d3_ng2_service__["a" /* D3Service */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1_d3_ng2_service__["a" /* D3Service */]) === "function" && _c || Object, typeof (_d = typeof __WEBPACK_IMPORTED_MODULE_2__graph_communication_service__["a" /* GraphCommunicationService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_2__graph_communication_service__["a" /* GraphCommunicationService */]) === "function" && _d || Object, typeof (_e = typeof __WEBPACK_IMPORTED_MODULE_4__event_communication_service__["a" /* EventCommunicationService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_4__event_communication_service__["a" /* EventCommunicationService */]) === "function" && _e || Object])
], GraphLegendComponent);

var _a, _b, _c, _d, _e;
//# sourceMappingURL=graph-legend.component.js.map

/***/ }),

/***/ "../../../../../src/app/graph/graph-main/graph-main.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, "input[type=checkbox] {\n  vertical-align: middle;\n}\n\n.controls {\n  font-size: 14px;\n}\n\n.buttonLabel {\n  font-weight: bold;\n}\n\n#graphOptions::before {\n  font-weight: bold;\n  display: inline-block;\n  content: \"Options: \";\n  font-size: 12px;\n  width: 70px;\n}\n\n#graphOptions {\n  min-height: 24px;\n}\n\n\n#legend::before {\n  font-weight: bold;\n  display: inline-block;\n  content: \"Types: \";\n  font-size: 12px;\n  width: 70px;\n}\n", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/graph/graph-main/graph-main.component.html":
/***/ (function(module, exports) {

module.exports = "<ngb-tabset (tabChange)=\"hierarchyTabChanged($event)\" >\n  <ngb-tab *ngFor=\"let tab of tabs\" id=\"tab-{{tab.name}}\"  title=\"{{tab.name}}\" [disabled]=\"!tab.active\">\n    <ng-template ngbTabContent></ng-template>\n  </ngb-tab>\n</ngb-tabset>\n\n<app-graph-search [graph]=\"graph\"></app-graph-search>\n\n<span *ngIf=\"!showHierarchicalGraph\">\n  <button class=\"btn btn-secondary btn-sm\" (click)=\"resetZoom()\">Reset Zoom</button>\n</span>\n<span *ngIf=\"showHierarchicalGraph\">\n  <button class=\"btn btn-secondary btn-sm\" (click)=\"expandAll()\">Expand All</button>\n  <button class=\"btn btn-secondary btn-sm\" (click)=\"collapseAll()\">Collapse All</button>\n  <button class=\"btn btn-secondary btn-sm\" (click)=\"resetZoom()\">Reset Zoom</button>\n</span>\n\n\n<div class=\"controls\" id=\"graphOptions\">\n  <span *ngIf=\"!showHierarchicalGraph\">\n    <label class=\"buttonLabel\"><input type=\"checkbox\" [(ngModel)]=\"layoutActive\" (change)=\"layoutActiveTriggered()\" >layout active</label>\n    <label class=\"buttonLabel\"><input type=\"checkbox\" [(ngModel)]=\"nodeCollision\" [disabled]=\"!nodeCollisionActive\">node collision</label>\n    <label class=\"buttonLabel\"><input type=\"checkbox\" [(ngModel)]=\"allLinks\">all Links</label>\n    <label class=\"buttonLabel\"><input type=\"checkbox\" [(ngModel)]=\"clickToAdd\">add/remove on click</label>\n  </span>\n\n  <span *ngIf=\"showHierarchicalGraph\">\n  </span>\n\n</div>\n\n\n<app-graph-legend [graph]=\"graph\" [isHierarchical]=showHierarchicalGraph></app-graph-legend>\n\n<app-graph-canvas\n        [graph]=graph\n        [isHierarchical]=showHierarchicalGraph\n        [layoutActive]=layoutActive\n        [nodeCollision]=nodeCollision\n        [allLinks]=allLinks\n        [clickToAdd]=clickToAdd\n></app-graph-canvas>\n\n"

/***/ }),

/***/ "../../../../../src/app/graph/graph-main/graph-main.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__ = __webpack_require__("../../../../../src/app/graph/helper/graph-data.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__helper_tab__ = __webpack_require__("../../../../../src/app/graph/helper/tab.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__ng_bootstrap_ng_bootstrap__ = __webpack_require__("../../../../@ng-bootstrap/ng-bootstrap/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__event_communication_service__ = __webpack_require__("../../../../../src/app/graph/event-communication.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5__selected_project_service__ = __webpack_require__("../../../../../src/app/selected-project.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return GraphMainComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};






var GraphMainComponent = (function () {
    function GraphMainComponent(eventCommunicationService, selectedProjectService) {
        var _this = this;
        this.eventCommunicationService = eventCommunicationService;
        this.selectedProjectService = selectedProjectService;
        this.tabs = [];
        this.layoutActive = true;
        this.nodeCollision = true;
        this.allLinks = false;
        this.clickToAdd = false;
        this.nodeCollisionActive = true;
        this.showHierarchicalGraph = false;
        eventCommunicationService.itemSelected$.subscribe(function () { return _this.hierarchyTabs.select('tab-Flat'); });
    }
    Object.defineProperty(GraphMainComponent.prototype, "graph", {
        get: function () {
            return this._graph;
        },
        set: function (graph) {
            this._graph = graph;
            if (graph != null) {
                this.tabs = [];
                this.setUpTabs();
            }
        },
        enumerable: true,
        configurable: true
    });
    GraphMainComponent.prototype.ngOnInit = function () {
    };
    GraphMainComponent.prototype.setUpTabs = function () {
        this.setUpFlatTab();
        this.setUpHierarchyTab();
        this.hierarchyTabs.select('tab-Flat');
        this.showHierarchicalGraph = false;
    };
    GraphMainComponent.prototype.setUpFlatTab = function () {
        var caption = 'Flat';
        var data = this.graph;
        this.tabs.push(new __WEBPACK_IMPORTED_MODULE_2__helper_tab__["a" /* Tab */](caption, data));
    };
    GraphMainComponent.prototype.setUpHierarchyTab = function () {
        var _this = this;
        var caption = 'Hierarchy';
        var data = this.graph;
        this.getGroupItemType().then(function (groupItemType) {
            var topElements = data.elements.some(function (e) {
                return e.type === groupItemType;
            });
            _this.tabs.push(new __WEBPACK_IMPORTED_MODULE_2__helper_tab__["a" /* Tab */](caption, data, topElements, true));
        });
    };
    GraphMainComponent.prototype.resetZoom = function () {
        this.eventCommunicationService.resetZoomClicked();
    };
    GraphMainComponent.prototype.collapseAll = function () {
        this.eventCommunicationService.collapseAllClicked();
    };
    GraphMainComponent.prototype.expandAll = function () {
        this.eventCommunicationService.expandAllClicked();
    };
    GraphMainComponent.prototype.hierarchyTabChanged = function (event) {
        switch (event.nextId) {
            case 'tab-Flat':
                this.showHierarchicalGraph = false;
                break;
            case 'tab-Hierarchy':
                this.showHierarchicalGraph = true;
                break;
            default: console.log('Tab Error');
        }
    };
    GraphMainComponent.prototype.layoutActiveTriggered = function () {
        this.nodeCollision = this.layoutActive;
        this.nodeCollisionActive = this.layoutActive;
    };
    GraphMainComponent.prototype.getGroupItemType = function () {
        var _this = this;
        return new Promise(function (resolve, reject) {
            _this.selectedProjectService.getProperties()
                .subscribe(function (properties) {
                // if(properties.graph.groupItemType != '') {
                resolve(properties['graph'].groupItemType);
                // }
                // resolve('');
            }, function (error) {
                reject(error);
            });
        });
    };
    return GraphMainComponent;
}());
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["ViewChild"])(__WEBPACK_IMPORTED_MODULE_3__ng_bootstrap_ng_bootstrap__["c" /* NgbTabset */]),
    __metadata("design:type", typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_3__ng_bootstrap_ng_bootstrap__["c" /* NgbTabset */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_3__ng_bootstrap_ng_bootstrap__["c" /* NgbTabset */]) === "function" && _a || Object)
], GraphMainComponent.prototype, "hierarchyTabs", void 0);
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */]) === "function" && _b || Object),
    __metadata("design:paramtypes", [typeof (_c = typeof __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */]) === "function" && _c || Object])
], GraphMainComponent.prototype, "graph", null);
GraphMainComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-graph-main',
        template: __webpack_require__("../../../../../src/app/graph/graph-main/graph-main.component.html"),
        styles: [__webpack_require__("../../../../../src/app/graph/graph-main/graph-main.component.css")],
        encapsulation: __WEBPACK_IMPORTED_MODULE_0__angular_core__["ViewEncapsulation"].None
    }),
    __metadata("design:paramtypes", [typeof (_d = typeof __WEBPACK_IMPORTED_MODULE_4__event_communication_service__["a" /* EventCommunicationService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_4__event_communication_service__["a" /* EventCommunicationService */]) === "function" && _d || Object, typeof (_e = typeof __WEBPACK_IMPORTED_MODULE_5__selected_project_service__["a" /* SelectedProjectService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_5__selected_project_service__["a" /* SelectedProjectService */]) === "function" && _e || Object])
], GraphMainComponent);

var _a, _b, _c, _d, _e;
//# sourceMappingURL=graph-main.component.js.map

/***/ }),

/***/ "../../../../../src/app/graph/graph-main/graph-search/graph-search.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, ".awesomplete li {\n  border: 1px solid white;\n}\n.awesomplete li.selected {\n  border: 1px solid crimson;\n}\n\n.awesomplete li > div:first-child {\n  display: -webkit-box;\n  display: -ms-flexbox;\n  display: flex;\n}\n\n.awesomplete .name {\n  -webkit-box-flex: 1;\n      -ms-flex: 1;\n          flex: 1;\n}\n\n.awesomplete li:hover .name {\n  text-decoration: underline;\n}\n\n.awesomplete .fullName {\n  font-size: 9px;\n}\n\n.awesomplete .addButton {\n  width: 20px;\n  border: 1px solid gray;\n  background: lightgray;\n  text-align: center;\n}\n\n.awesomplete .addButton:hover {\n  background: white;\n}\n\n#search {\n  width: 20em;\n}\n", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/graph/graph-main/graph-search/graph-search.component.html":
/***/ (function(module, exports) {

module.exports = "<span class=\"controls\">\n  <input id=\"search\" type=\"search\" placeholder=\"search\" />\n</span>\n"

/***/ }),

/***/ "../../../../../src/app/graph/graph-main/graph-search/graph-search.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__ = __webpack_require__("../../../../../src/app/graph/helper/graph-data.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2_d3_ng2_service__ = __webpack_require__("../../../../d3-ng2-service/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__graph_communication_service__ = __webpack_require__("../../../../../src/app/graph/graph-communication.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__helper_graph_helper_service__ = __webpack_require__("../../../../../src/app/graph/helper/graph-helper.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5_awesomplete__ = __webpack_require__("../../../../awesomplete/awesomplete.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5_awesomplete___default = __webpack_require__.n(__WEBPACK_IMPORTED_MODULE_5_awesomplete__);
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_6__event_communication_service__ = __webpack_require__("../../../../../src/app/graph/event-communication.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_7__helper_SearchEvent__ = __webpack_require__("../../../../../src/app/graph/helper/SearchEvent.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return GraphSearchComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};








var GraphSearchComponent = (function () {
    function GraphSearchComponent(d3Service, graphCommunicationService, eventCommunicationService) {
        this.graphCommunicationService = graphCommunicationService;
        this.eventCommunicationService = eventCommunicationService;
        this.d3 = d3Service.getD3();
    }
    Object.defineProperty(GraphSearchComponent.prototype, "graph", {
        get: function () {
            return this._graph;
        },
        set: function (graph) {
            this._graph = graph;
            if (graph != null) {
                this.resetSearch();
                this.setUpSearch();
            }
        },
        enumerable: true,
        configurable: true
    });
    GraphSearchComponent.prototype.ngOnInit = function () {
    };
    GraphSearchComponent.prototype.resetSearch = function () {
        if (document.getElementById('main')) {
            var controls = document.getElementById('main').getElementsByClassName('controls');
            for (var i = 0; i < controls.length; i++) {
                var awesomplete = controls[i].getElementsByClassName('awesomplete');
                if (controls[i].getElementsByClassName('awesomplete').length > 0) {
                    var element = controls[i];
                    element.removeChild(awesomplete[0]);
                    var search = document.createElement('input');
                    search.setAttribute('id', 'search');
                    search.setAttribute('type', 'search');
                    search.setAttribute('placeholder', 'search');
                    element.appendChild(search);
                }
            }
        }
    };
    GraphSearchComponent.prototype.setUpSearch = function () {
        var _this = this;
        var graph = this.graph;
        var d3 = this.d3;
        var graphCommunicationService = this.graphCommunicationService;
        if (document.getElementById('search')) {
            var search = document.getElementById('search');
            /* global Awesomplete */
            var awe = new __WEBPACK_IMPORTED_MODULE_5_awesomplete__(search, {
                minChars: 1,
                list: graph.elements,
                filter: function (item, input) {
                    return __WEBPACK_IMPORTED_MODULE_4__helper_graph_helper_service__["a" /* GraphHelperService */].getNodeName(item.value).toLowerCase().indexOf(input.toLowerCase()) > -1;
                },
                sort: function (a, b) {
                    var nameA = __WEBPACK_IMPORTED_MODULE_4__helper_graph_helper_service__["a" /* GraphHelperService */].getNodeName(a.value), nameB = __WEBPACK_IMPORTED_MODULE_4__helper_graph_helper_service__["a" /* GraphHelperService */].getNodeName(b.value);
                    return nameA < nameB ? -1 : nameA > nameB ? 1 : 0;
                },
                item: function (item, input) {
                    var li = d3.select(document.createElement('li'));
                    var nodeTypes = graphCommunicationService.nodeTypes;
                    li.style('background', nodeTypes[item.value.type]);
                    var name = __WEBPACK_IMPORTED_MODULE_4__helper_graph_helper_service__["a" /* GraphHelperService */].getNodeName(item.value);
                    var text = input === '' ? name : name.replace(new RegExp(__WEBPACK_IMPORTED_MODULE_5_awesomplete__["$"].regExpEscape(input.trim()), 'gi'), '<mark>$&</mark>');
                    var selectedNodes = graphCommunicationService.selectedNodes;
                    var isSelected = selectedNodes.some(function (n) {
                        return n.id === item.value.id;
                    });
                    if (isSelected) {
                        li.classed('selected', true);
                    }
                    var nameElement = li.append('div');
                    nameElement.append('div').classed('name', true)
                        .html(text);
                    nameElement.append('div').classed('addButton', true)
                        .html(isSelected ? '&minus;' : '+')
                        .attr('title', isSelected ? 'remove from selection' : 'add to selection');
                    li.append('div').classed('fullName', true)
                        .html(item.value.name);
                    return li.node();
                },
                replace: function () {
                    this.input.value = '';
                }
            });
            search.addEventListener('awesomplete-select', function (e) {
                var selectEvent = new __WEBPACK_IMPORTED_MODULE_7__helper_SearchEvent__["a" /* SearchEvent */](e['text'].value, e['origin'].className === 'addButton');
                _this.eventCommunicationService.selectSearchItem(selectEvent);
            });
        }
    };
    return GraphSearchComponent;
}());
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */]) === "function" && _a || Object),
    __metadata("design:paramtypes", [typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */]) === "function" && _b || Object])
], GraphSearchComponent.prototype, "graph", null);
GraphSearchComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-graph-search',
        template: __webpack_require__("../../../../../src/app/graph/graph-main/graph-search/graph-search.component.html"),
        styles: [__webpack_require__("../../../../../src/app/graph/graph-main/graph-search/graph-search.component.css")],
        encapsulation: __WEBPACK_IMPORTED_MODULE_0__angular_core__["ViewEncapsulation"].None
    }),
    __metadata("design:paramtypes", [typeof (_c = typeof __WEBPACK_IMPORTED_MODULE_2_d3_ng2_service__["a" /* D3Service */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_2_d3_ng2_service__["a" /* D3Service */]) === "function" && _c || Object, typeof (_d = typeof __WEBPACK_IMPORTED_MODULE_3__graph_communication_service__["a" /* GraphCommunicationService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_3__graph_communication_service__["a" /* GraphCommunicationService */]) === "function" && _d || Object, typeof (_e = typeof __WEBPACK_IMPORTED_MODULE_6__event_communication_service__["a" /* EventCommunicationService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_6__event_communication_service__["a" /* EventCommunicationService */]) === "function" && _e || Object])
], GraphSearchComponent);

var _a, _b, _c, _d, _e;
//# sourceMappingURL=graph-search.component.js.map

/***/ }),

/***/ "../../../../../src/app/graph/graph-routing.module.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__angular_router__ = __webpack_require__("../../../router/@angular/router.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__graph_graph_component__ = __webpack_require__("../../../../../src/app/graph/graph/graph.component.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return GraphRoutingModule; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};



var routes = [
    { path: '', component: __WEBPACK_IMPORTED_MODULE_2__graph_graph_component__["a" /* GraphComponent */] }
];
var GraphRoutingModule = (function () {
    function GraphRoutingModule() {
    }
    return GraphRoutingModule;
}());
GraphRoutingModule = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["NgModule"])({
        imports: [__WEBPACK_IMPORTED_MODULE_1__angular_router__["a" /* RouterModule */].forChild(routes)],
        exports: [__WEBPACK_IMPORTED_MODULE_1__angular_router__["a" /* RouterModule */]]
    })
], GraphRoutingModule);

//# sourceMappingURL=graph-routing.module.js.map

/***/ }),

/***/ "../../../../../src/app/graph/graph-tree/graph-tree.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, "", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/graph/graph-tree/graph-tree.component.html":
/***/ (function(module, exports) {

module.exports = "<ngb-tabset>\n  <ngb-tab *ngFor=\"let tab of tabs\" title=\"{{tab.name}}\">\n    <ng-template ngbTabContent>\n      <app-tab-content [tree]=tab.content></app-tab-content>\n    </ng-template>\n\n  </ngb-tab>\n\n</ngb-tabset>\n"

/***/ }),

/***/ "../../../../../src/app/graph/graph-tree/graph-tree.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__ = __webpack_require__("../../../../../src/app/graph/helper/graph-data.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__helper_tab__ = __webpack_require__("../../../../../src/app/graph/helper/tab.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__tab_tree__ = __webpack_require__("../../../../../src/app/graph/graph-tree/tab-tree.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__helper_graph_helper_service__ = __webpack_require__("../../../../../src/app/graph/helper/graph-helper.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5__selected_project_service__ = __webpack_require__("../../../../../src/app/selected-project.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return GraphTreeComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};






var GraphTreeComponent = (function () {
    function GraphTreeComponent(selectedProjectService) {
        this.selectedProjectService = selectedProjectService;
        this.tabs = [];
    }
    Object.defineProperty(GraphTreeComponent.prototype, "graph", {
        get: function () {
            return this._graph;
        },
        set: function (graph) {
            this._graph = graph;
            if (graph != null) {
                this.tabs = [];
                this.setUpTabs();
            }
        },
        enumerable: true,
        configurable: true
    });
    GraphTreeComponent.prototype.ngOnInit = function () {
    };
    GraphTreeComponent.prototype.setUpTabs = function () {
        var _this = this;
        this.selectedProjectService.getProperties().subscribe(function (prop) {
            prop.graph.tabs.forEach(function (tab) {
                var name = tab.name;
                var rootType = tab.rootType;
                var linkList = tab.linkList;
                var flatTypes = tab.flatTypes;
                _this.setUpTab(name, rootType, linkList, flatTypes);
            });
        });
    };
    GraphTreeComponent.prototype.setUpTab = function (tabName, rootType, linkTypes, flatTypes) {
        var graph = this.graph;
        var links = graph.links.filter(function (l) {
            for (var _i = 0, linkTypes_1 = linkTypes; _i < linkTypes_1.length; _i++) {
                var linkType = linkTypes_1[_i];
                if (l.forwardLinks[linkType]) {
                    return true;
                }
            }
            return false;
        });
        var elements = graph.elements.filter(function (e) {
            return e.type === rootType && !links.some(function (l) {
                return l.target === e;
            });
        });
        // Flat View if there is no Hierarchy present
        if (elements.length === 0 && flatTypes) {
            elements = graph.elements.filter(function (e) {
                for (var _i = 0, flatTypes_1 = flatTypes; _i < flatTypes_1.length; _i++) {
                    var flatType = flatTypes_1[_i];
                    if (e.type === flatType) {
                        return true;
                    }
                }
                return false;
            });
        }
        this.createTreeTab(tabName, elements, links);
    };
    GraphTreeComponent.prototype.createTreeTab = function (caption, items, links) {
        var data = this.createTree(items, links);
        this.tabs.push(new __WEBPACK_IMPORTED_MODULE_2__helper_tab__["a" /* Tab */](caption, data));
    };
    GraphTreeComponent.prototype.createTree = function (items, links) {
        var tree = new __WEBPACK_IMPORTED_MODULE_3__tab_tree__["a" /* TabTree */](null);
        for (var _i = 0, items_1 = items; _i < items_1.length; _i++) {
            var item = items_1[_i];
            var newTreeElement = tree.addChild(item);
            this.createSubTree(newTreeElement, links);
        }
        return tree;
    };
    GraphTreeComponent.prototype.createSubTree = function (root, links) {
        var childrenLinks = links.filter(function (l) {
            return __WEBPACK_IMPORTED_MODULE_4__helper_graph_helper_service__["a" /* GraphHelperService */].elementEquals(root.element, l.source);
        });
        for (var _i = 0, childrenLinks_1 = childrenLinks; _i < childrenLinks_1.length; _i++) {
            var link = childrenLinks_1[_i];
            var child = root.addChild(link.target);
            this.createSubTree(child, links);
        }
    };
    return GraphTreeComponent;
}());
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */]) === "function" && _a || Object),
    __metadata("design:paramtypes", [typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__helper_graph_data__["a" /* GraphData */]) === "function" && _b || Object])
], GraphTreeComponent.prototype, "graph", null);
GraphTreeComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-graph-tree',
        template: __webpack_require__("../../../../../src/app/graph/graph-tree/graph-tree.component.html"),
        styles: [__webpack_require__("../../../../../src/app/graph/graph-tree/graph-tree.component.css")]
    }),
    __metadata("design:paramtypes", [typeof (_c = typeof __WEBPACK_IMPORTED_MODULE_5__selected_project_service__["a" /* SelectedProjectService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_5__selected_project_service__["a" /* SelectedProjectService */]) === "function" && _c || Object])
], GraphTreeComponent);

var _a, _b, _c;
//# sourceMappingURL=graph-tree.component.js.map

/***/ }),

/***/ "../../../../../src/app/graph/graph-tree/tab-content/tab-content.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, "hr {\n  margin: 0 0;\n}\n\nul {\n  list-style-type: none;\n  margin: 0;\n}\n\n.treeView {\n  -webkit-box-flex: 1;\n      -ms-flex: 1;\n          flex: 1;\n  overflow: scroll;\n  padding-top: 2px;\n  height: 86vh;\n}\n\n.treeView ul {\n  display: inline-block;\n  padding: 0;\n  white-space: nowrap;\n}\n\n.treeView li {\n  padding-top: 1px;\n}\n\n.treeView ul ul {\n  margin-left: 9px;\n  border-left: 1px dotted black;\n  padding-left: 8px;\n}\n\n.treeView li > div {\n  display: -webkit-box;\n  display: -ms-flexbox;\n  display: flex;\n}\n\n.treeButton {\n  margin: 2px;\n  width: 14px;\n  height: 14px;\n  border: 1px solid black;\n  background-color: lightgray;\n  text-align: center;\n  font-size: 11px;\n  cursor: default;\n}\n\n.treeNodeCaption {\n  border: 1px solid white;\n  padding: 2px;\n  cursor: pointer;\n}\n.treeNodeCaption:hover {\n  text-decoration: underline;\n}\n.treeNodeCaption.selected {\n  border: 1px solid crimson;\n}\n", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/graph/graph-tree/tab-content/tab-content.component.html":
/***/ (function(module, exports) {

module.exports = "<button type=\"button\" class=\"btn btn-secondary btn-sm\" (click)=\"expandTab()\"\n        placement=\"right\" ngbTooltip=\"Expand Tab\">\n  <i class=\"fa fa-plus\" aria-hidden=\"true\"></i>\n</button>\n\n<button type=\"button\" class=\"btn btn-secondary btn-sm\" (click)=\"collapseTab()\"\n        placement=\"right\" ngbTooltip=\"Collapse Tab \">\n  <i class=\"fa fa-minus\" aria-hidden=\"true\"></i>\n</button>\n\n<hr/>\n\n<div class=\"treeView\">\n\n</div>\n"

/***/ }),

/***/ "../../../../../src/app/graph/graph-tree/tab-content/tab-content.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__tab_tree__ = __webpack_require__("../../../../../src/app/graph/graph-tree/tab-tree.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2_d3_ng2_service__ = __webpack_require__("../../../../d3-ng2-service/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__helper_graph_helper_service__ = __webpack_require__("../../../../../src/app/graph/helper/graph-helper.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__event_communication_service__ = __webpack_require__("../../../../../src/app/graph/event-communication.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5__graph_communication_service__ = __webpack_require__("../../../../../src/app/graph/graph-communication.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return TabContentComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};






/**
 * Nur Darstellung, bekommt direkt Daten
 */
var TabContentComponent = (function () {
    function TabContentComponent(element, d3Service, eventCommunicationService, graphCommunicationService) {
        this.eventCommunicationService = eventCommunicationService;
        this.graphCommunicationService = graphCommunicationService;
        this.d3 = d3Service.getD3();
        this.parentNativeElement = element.nativeElement;
    }
    Object.defineProperty(TabContentComponent.prototype, "tree", {
        get: function () {
            return this._tree;
        },
        set: function (tree) {
            this._tree = tree;
            if (tree != null) {
                this.updateTab();
            }
        },
        enumerable: true,
        configurable: true
    });
    ;
    TabContentComponent.prototype.ngOnInit = function () {
    };
    TabContentComponent.prototype.updateTab = function () {
        var tree = this.tree;
        var d3 = this.d3;
        var d3ParentElement;
        if (this.parentNativeElement != null) {
            d3ParentElement = d3.select(this.parentNativeElement);
            var tab = d3ParentElement.select('.treeView');
            this.createTreeView(tree.children, tab);
        }
    };
    TabContentComponent.prototype.createTreeView = function (items, parent) {
        var _this = this;
        var d3 = this.d3;
        var itemList = items.map(function (i) {
            return i.element;
        });
        var elements = parent.append('ul').selectAll('li')
            .data(itemList).enter()
            .append('li')
            .sort(function (a, b) {
            if (a.type === b.type) {
                if (__WEBPACK_IMPORTED_MODULE_3__helper_graph_helper_service__["a" /* GraphHelperService */].getNodeName(a) < __WEBPACK_IMPORTED_MODULE_3__helper_graph_helper_service__["a" /* GraphHelperService */].getNodeName(b)) {
                    return -1;
                }
                else if (__WEBPACK_IMPORTED_MODULE_3__helper_graph_helper_service__["a" /* GraphHelperService */].getNodeName(b) < __WEBPACK_IMPORTED_MODULE_3__helper_graph_helper_service__["a" /* GraphHelperService */].getNodeName(a)) {
                    return 1;
                }
                return 0;
            }
            var sortOrder = Object.keys(_this.graphCommunicationService.nodeTypes);
            return sortOrder.indexOf(a.type) - sortOrder.indexOf(b.type);
        });
        var nodeContent = elements.append('div');
        nodeContent.append('div').style('background', function (d) {
            return _this.graphCommunicationService.nodeTypes[d.type];
        }).text(function (d) {
            return __WEBPACK_IMPORTED_MODULE_3__helper_graph_helper_service__["a" /* GraphHelperService */].getNodeName(d);
        }).on('click', function (d) {
            _this.eventCommunicationService.selectTreeItem(d);
        }).classed('treeNodeCaption', true);
        var object = this;
        elements.each(function (d) {
            var element = items.find(function (i) {
                return __WEBPACK_IMPORTED_MODULE_3__helper_graph_helper_service__["a" /* GraphHelperService */].elementEquals(i.element, d);
            });
            var subItems = element.children;
            if (subItems.length) {
                var item_1 = d3.select(this);
                item_1.select('div').insert('div', '.treeNodeCaption')
                    .attr('class', 'treeButton')
                    .html('&minus;')
                    .on('click', function () {
                    var subList = item_1.select('ul'), me = d3.select(this), status = subList.style('display');
                    subList.style('display', status === 'none' ? null : 'none');
                    me.html(status === 'none' ? '&minus;' : '+');
                });
                object.createTreeView(subItems, d3.select(this));
            }
        });
        this.collapseTab();
    };
    TabContentComponent.prototype.collapseTab = function () {
        var root = this.d3.select(this.parentNativeElement).select('.treeView').select('ul');
        root.selectAll('ul').style('display', 'none');
        root.selectAll('div').select('.treeButton').html('+');
    };
    TabContentComponent.prototype.expandTab = function () {
        var root = this.d3.select(this.parentNativeElement).select('.treeView').select('ul');
        root.selectAll('ul').style('display', null);
        root.selectAll('div').select('.treeButton').html('&minus;');
    };
    return TabContentComponent;
}());
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__tab_tree__["a" /* TabTree */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__tab_tree__["a" /* TabTree */]) === "function" && _a || Object),
    __metadata("design:paramtypes", [typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_1__tab_tree__["a" /* TabTree */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__tab_tree__["a" /* TabTree */]) === "function" && _b || Object])
], TabContentComponent.prototype, "tree", null);
TabContentComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-tab-content',
        template: __webpack_require__("../../../../../src/app/graph/graph-tree/tab-content/tab-content.component.html"),
        styles: [__webpack_require__("../../../../../src/app/graph/graph-tree/tab-content/tab-content.component.css")],
        encapsulation: __WEBPACK_IMPORTED_MODULE_0__angular_core__["ViewEncapsulation"].None,
    }),
    __metadata("design:paramtypes", [typeof (_c = typeof __WEBPACK_IMPORTED_MODULE_0__angular_core__["ElementRef"] !== "undefined" && __WEBPACK_IMPORTED_MODULE_0__angular_core__["ElementRef"]) === "function" && _c || Object, typeof (_d = typeof __WEBPACK_IMPORTED_MODULE_2_d3_ng2_service__["a" /* D3Service */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_2_d3_ng2_service__["a" /* D3Service */]) === "function" && _d || Object, typeof (_e = typeof __WEBPACK_IMPORTED_MODULE_4__event_communication_service__["a" /* EventCommunicationService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_4__event_communication_service__["a" /* EventCommunicationService */]) === "function" && _e || Object, typeof (_f = typeof __WEBPACK_IMPORTED_MODULE_5__graph_communication_service__["a" /* GraphCommunicationService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_5__graph_communication_service__["a" /* GraphCommunicationService */]) === "function" && _f || Object])
], TabContentComponent);

var _a, _b, _c, _d, _e, _f;
//# sourceMappingURL=tab-content.component.js.map

/***/ }),

/***/ "../../../../../src/app/graph/graph-tree/tab-tree.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return TabTree; });
var TabTree = (function () {
    function TabTree(element) {
        this._element = element;
        this._children = [];
    }
    Object.defineProperty(TabTree.prototype, "element", {
        get: function () {
            return this._element;
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(TabTree.prototype, "children", {
        get: function () {
            return this._children;
        },
        enumerable: true,
        configurable: true
    });
    // Add Child to Tree, returns the new TreeElement;
    TabTree.prototype.addChild = function (child) {
        var newLength = this._children.push(new TabTree(child));
        return this._children[newLength - 1];
    };
    return TabTree;
}());

//# sourceMappingURL=tab-tree.js.map

/***/ }),

/***/ "../../../../../src/app/graph/graph.module.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
Object.defineProperty(__webpack_exports__, "__esModule", { value: true });
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__angular_common__ = __webpack_require__("../../../common/@angular/common.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__graph_graph_component__ = __webpack_require__("../../../../../src/app/graph/graph/graph.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__graph_routing_module__ = __webpack_require__("../../../../../src/app/graph/graph-routing.module.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__selected_project_service__ = __webpack_require__("../../../../../src/app/selected-project.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5__selected_od_service__ = __webpack_require__("../../../../../src/app/selected-od.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_6_d3_ng2_service__ = __webpack_require__("../../../../d3-ng2-service/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_7__graph_tree_graph_tree_component__ = __webpack_require__("../../../../../src/app/graph/graph-tree/graph-tree.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_8__graph_main_graph_main_component__ = __webpack_require__("../../../../../src/app/graph/graph-main/graph-main.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_9__helper_graph_helper_service__ = __webpack_require__("../../../../../src/app/graph/helper/graph-helper.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_10__ng_bootstrap_ng_bootstrap__ = __webpack_require__("../../../../@ng-bootstrap/ng-bootstrap/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_11__graph_tree_tab_content_tab_content_component__ = __webpack_require__("../../../../../src/app/graph/graph-tree/tab-content/tab-content.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_12__event_communication_service__ = __webpack_require__("../../../../../src/app/graph/event-communication.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_13__graph_main_graph_canvas_graph_canvas_component__ = __webpack_require__("../../../../../src/app/graph/graph-main/graph-canvas/graph-canvas.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_14__angular_forms__ = __webpack_require__("../../../forms/@angular/forms.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_15__graph_communication_service__ = __webpack_require__("../../../../../src/app/graph/graph-communication.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_16__graph_main_graph_search_graph_search_component__ = __webpack_require__("../../../../../src/app/graph/graph-main/graph-search/graph-search.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_17__graph_main_graph_legend_graph_legend_component__ = __webpack_require__("../../../../../src/app/graph/graph-main/graph-legend/graph-legend.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_18__helper_graph_canvas_helper_service__ = __webpack_require__("../../../../../src/app/graph/helper/graph-canvas-helper.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_19__helper_graph_tree_service__ = __webpack_require__("../../../../../src/app/graph/helper/graph-tree.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_20__color_service__ = __webpack_require__("../../../../../src/app/color.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "GraphModule", function() { return GraphModule; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};





















var GraphModule = GraphModule_1 = (function () {
    function GraphModule() {
    }
    GraphModule.forRoot = function () {
        return {
            ngModule: GraphModule_1,
            providers: [
                __WEBPACK_IMPORTED_MODULE_4__selected_project_service__["a" /* SelectedProjectService */],
                __WEBPACK_IMPORTED_MODULE_5__selected_od_service__["a" /* SelectedOdService */],
                __WEBPACK_IMPORTED_MODULE_6_d3_ng2_service__["a" /* D3Service */],
                __WEBPACK_IMPORTED_MODULE_20__color_service__["a" /* ColorService */]
            ]
        };
    };
    return GraphModule;
}());
GraphModule = GraphModule_1 = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["NgModule"])({
        imports: [
            __WEBPACK_IMPORTED_MODULE_1__angular_common__["CommonModule"],
            __WEBPACK_IMPORTED_MODULE_3__graph_routing_module__["a" /* GraphRoutingModule */],
            __WEBPACK_IMPORTED_MODULE_10__ng_bootstrap_ng_bootstrap__["a" /* NgbModule */],
            __WEBPACK_IMPORTED_MODULE_14__angular_forms__["a" /* FormsModule */]
        ],
        declarations: [
            __WEBPACK_IMPORTED_MODULE_2__graph_graph_component__["a" /* GraphComponent */],
            __WEBPACK_IMPORTED_MODULE_7__graph_tree_graph_tree_component__["a" /* GraphTreeComponent */],
            __WEBPACK_IMPORTED_MODULE_8__graph_main_graph_main_component__["a" /* GraphMainComponent */],
            __WEBPACK_IMPORTED_MODULE_11__graph_tree_tab_content_tab_content_component__["a" /* TabContentComponent */],
            __WEBPACK_IMPORTED_MODULE_13__graph_main_graph_canvas_graph_canvas_component__["a" /* GraphCanvasComponent */],
            __WEBPACK_IMPORTED_MODULE_16__graph_main_graph_search_graph_search_component__["a" /* GraphSearchComponent */],
            __WEBPACK_IMPORTED_MODULE_17__graph_main_graph_legend_graph_legend_component__["a" /* GraphLegendComponent */]
        ],
        providers: [
            __WEBPACK_IMPORTED_MODULE_9__helper_graph_helper_service__["a" /* GraphHelperService */],
            __WEBPACK_IMPORTED_MODULE_12__event_communication_service__["a" /* EventCommunicationService */],
            __WEBPACK_IMPORTED_MODULE_15__graph_communication_service__["a" /* GraphCommunicationService */],
            __WEBPACK_IMPORTED_MODULE_18__helper_graph_canvas_helper_service__["a" /* GraphCanvasHelperService */],
            __WEBPACK_IMPORTED_MODULE_19__helper_graph_tree_service__["a" /* GraphTreeService */]
        ]
    })
], GraphModule);

var GraphModule_1;
//# sourceMappingURL=graph.module.js.map

/***/ }),

/***/ "../../../../../src/app/graph/graph/graph.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, ".graph-body {\n  height: 90vh;\n  font-family: sans-serif;\n  margin: 0;\n  display: -webkit-box;\n  display: -ms-flexbox;\n  display: flex;\n  -webkit-box-orient: horizontal;\n  -webkit-box-direction: normal;\n      -ms-flex-direction: row;\n          flex-direction: row;\n  -ms-flex-wrap: nowrap;\n      flex-wrap: nowrap\n}\n\n#tree {\n  -webkit-box-flex: 0;\n      -ms-flex: 0 0 256px;\n          flex: 0 0 256px;\n  display: -webkit-box;\n  display: -ms-flexbox;\n  display: flex;\n  -webkit-box-orient: vertical;\n  -webkit-box-direction: normal;\n      -ms-flex-direction: column;\n          flex-direction: column;\n  overflow: hidden;\n  font-size: 12px;\n}\n\n#splitter {\n  -webkit-box-flex: 0;\n      -ms-flex: 0 0 6px;\n          flex: 0 0 6px;\n  background: darkgray;\n  cursor: ew-resize;\n}\n\n#main {\n  -webkit-box-flex: 1;\n      -ms-flex: auto;\n          flex: auto;\n  -ms-flex-item-align: stretch;\n      align-self: stretch;\n  display: -webkit-box;\n  display: -ms-flexbox;\n  display: flex;\n  -webkit-box-orient: vertical;\n  -webkit-box-direction: normal;\n      -ms-flex-direction: column;\n          flex-direction: column;\n  overflow-x: scroll;\n  font-size: 12px;\n}\n", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/graph/graph/graph.component.html":
/***/ (function(module, exports) {

module.exports = "<div class=\"row graph-body\">\n\n  <div id=\"tree\">\n    <app-graph-tree [graph]=\"graph\">loading...</app-graph-tree>\n  </div>\n  <div id=\"splitter\" draggable=\"true\"></div>\n\n  <div id=\"main\">\n    <app-graph-main [graph]=\"graph\"></app-graph-main>\n  </div>\n\n\n</div>\n\n\n"

/***/ }),

/***/ "../../../../../src/app/graph/graph/graph.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__selected_od_service__ = __webpack_require__("../../../../../src/app/selected-od.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__helper_graph_helper_service__ = __webpack_require__("../../../../../src/app/graph/helper/graph-helper.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__helper_graph_data__ = __webpack_require__("../../../../../src/app/graph/helper/graph-data.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4_d3_ng2_service__ = __webpack_require__("../../../../d3-ng2-service/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5__graph_communication_service__ = __webpack_require__("../../../../../src/app/graph/graph-communication.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_6__event_communication_service__ = __webpack_require__("../../../../../src/app/graph/event-communication.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_7__color_service__ = __webpack_require__("../../../../../src/app/color.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return GraphComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};








var GraphComponent = (function () {
    function GraphComponent(d3Service, selectedOdService, graphCommunicationService, eventCommunicationService, colorService) {
        var _this = this;
        this.d3Service = d3Service;
        this.selectedOdService = selectedOdService;
        this.graphCommunicationService = graphCommunicationService;
        this.eventCommunicationService = eventCommunicationService;
        this.colorService = colorService;
        this.d3 = d3Service.getD3();
        selectedOdService.selectedOd$.subscribe(function () { return _this.update(); });
        colorService.domain$.subscribe(function (value) {
            _this.updateColors();
        });
    }
    GraphComponent.prototype.ngOnInit = function () {
        var _this = this;
        var treeView = document.getElementById('tree');
        var splitter = document.getElementById('splitter');
        var dragging = null;
        splitter.addEventListener('mousedown', function (e) {
            e.preventDefault();
            dragging = e.offsetX;
        });
        document.body.addEventListener('mousemove', function (e) {
            if (dragging != null) {
                var w = e.x - dragging;
                _this.d3.select(treeView).style('flex', '0 0 ' + w + 'px');
                _this.eventCommunicationService.splitterMoved();
            }
        });
        document.body.addEventListener('mouseup', function () {
            dragging = null;
        });
    };
    GraphComponent.prototype.ngAfterViewInit = function () {
        var _this = this;
        setTimeout(function () { return _this.update(); }, 0);
    };
    GraphComponent.prototype.update = function () {
        var _this = this;
        this.selectedOdService.getJsonData().subscribe(function (graph) {
            _this.graph = _this.graphDataChanged(graph);
        }, function (error) { return console.log(error); });
    };
    GraphComponent.prototype.graphDataChanged = function (graph) {
        var _this = this;
        var colorsUsed = 0;
        var d3 = this.d3;
        // let color: ScaleOrdinal<number, string> = d3.scaleOrdinal<number, string>(d3.schemeCategory20);
        var color = this.colorService.getColorScheme();
        this.graphCommunicationService.nodeTypes = {};
        var newGraph = new __WEBPACK_IMPORTED_MODULE_3__helper_graph_data__["a" /* GraphData */](graph);
        newGraph = __WEBPACK_IMPORTED_MODULE_2__helper_graph_helper_service__["a" /* GraphHelperService */].preProcessGraph(newGraph);
        newGraph.elements.forEach(function (e, i) {
            e.padding = 32;
            e.fixed = false;
            if (!_this.graphCommunicationService.nodeTypes[e.type]) {
                _this.graphCommunicationService.nodeTypes[e.type] = color[colorsUsed];
                colorsUsed = (colorsUsed + 1) % color.length;
            }
        });
        return newGraph;
    };
    GraphComponent.prototype.updateColors = function () {
        var _this = this;
        var colorsUsed = 0;
        var color = this.colorService.getColorScheme();
        var graph = this.graph;
        this.graph = undefined;
        if (graph) {
            this.graphCommunicationService.nodeTypes = [];
            this.graph.elements.forEach(function (e, i) {
                e.padding = 32;
                e.fixed = false;
                if (!_this.graphCommunicationService.nodeTypes[e.type]) {
                    _this.graphCommunicationService.nodeTypes[e.type] = color[colorsUsed];
                    colorsUsed = (colorsUsed + 1) % color.length;
                }
            });
        }
        setTimeout(function () {
            return _this.graph = graph;
        }, 0);
    };
    return GraphComponent;
}());
GraphComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-graph',
        template: __webpack_require__("../../../../../src/app/graph/graph/graph.component.html"),
        styles: [__webpack_require__("../../../../../src/app/graph/graph/graph.component.css")]
    }),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_4_d3_ng2_service__["a" /* D3Service */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_4_d3_ng2_service__["a" /* D3Service */]) === "function" && _a || Object, typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_1__selected_od_service__["a" /* SelectedOdService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__selected_od_service__["a" /* SelectedOdService */]) === "function" && _b || Object, typeof (_c = typeof __WEBPACK_IMPORTED_MODULE_5__graph_communication_service__["a" /* GraphCommunicationService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_5__graph_communication_service__["a" /* GraphCommunicationService */]) === "function" && _c || Object, typeof (_d = typeof __WEBPACK_IMPORTED_MODULE_6__event_communication_service__["a" /* EventCommunicationService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_6__event_communication_service__["a" /* EventCommunicationService */]) === "function" && _d || Object, typeof (_e = typeof __WEBPACK_IMPORTED_MODULE_7__color_service__["a" /* ColorService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_7__color_service__["a" /* ColorService */]) === "function" && _e || Object])
], GraphComponent);

var _a, _b, _c, _d, _e;
//# sourceMappingURL=graph.component.js.map

/***/ }),

/***/ "../../../../../src/app/graph/helper/SearchEvent.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return SearchEvent; });
var SearchEvent = (function () {
    function SearchEvent(item, addToSelection) {
        this.item = item;
        this.addToSelection = addToSelection;
    }
    return SearchEvent;
}());

//# sourceMappingURL=SearchEvent.js.map

/***/ }),

/***/ "../../../../../src/app/graph/helper/graph-canvas-helper.service.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__graph_tree_service__ = __webpack_require__("../../../../../src/app/graph/helper/graph-tree.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__graph_helper_service__ = __webpack_require__("../../../../../src/app/graph/helper/graph-helper.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__graph_data__ = __webpack_require__("../../../../../src/app/graph/helper/graph-data.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__graph_communication_service__ = __webpack_require__("../../../../../src/app/graph/graph-communication.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return GraphCanvasHelperService; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};





var GraphCanvasHelperService = GraphCanvasHelperService_1 = (function () {
    function GraphCanvasHelperService(graphTreeService, graphCommunicationService) {
        this.graphTreeService = graphTreeService;
        this.graphCommunicationService = graphCommunicationService;
    }
    GraphCanvasHelperService.closedNeighbourhood = function (graph, nodes, allLinks) {
        var result = { elements: [], links: [], groups: [] };
        nodes.forEach(function (n) {
            result.elements.push(n);
        });
        graph.links.forEach(function (l) {
            if (nodes.indexOf(l.source) > -1) {
                if (result.elements.indexOf(l.target) < 0) {
                    result.elements.push(l.target);
                }
                if (!allLinks) {
                    result.links.push(l);
                }
            }
            else if (nodes.indexOf(l.target) > -1) {
                if (result.elements.indexOf(l.source) < 0) {
                    result.elements.push(l.source);
                }
                if (!allLinks) {
                    result.links.push(l);
                }
            }
        });
        if (allLinks) {
            result.links = graph.links.filter(function (l) {
                return result.elements.indexOf(l.source) > -1 && result.elements.indexOf(l.target) > -1;
            });
        }
        return result;
    };
    GraphCanvasHelperService.getNodeId = function (node) {
        var id = node.isGroup ? 'g' + node.id : node.id;
        return node.inHierarchyGraph ? id + 'h' : id;
    };
    GraphCanvasHelperService.createContextMenu = function (contextMenu) {
        contextMenu.selectAll('*').remove();
        return contextMenu;
    };
    ;
    GraphCanvasHelperService.showContextMenu = function (contextMenu, e, coordinates) {
        e.preventDefault();
        // keep cm in window bounds
        var bBox = contextMenu.node().getBoundingClientRect();
        var x = Math.min(coordinates[0], window.innerWidth - bBox.width);
        var y = Math.min(coordinates[1], window.innerHeight - bBox.height);
        contextMenu.style('left', x + 'px').style('top', y + 'px');
    };
    ;
    GraphCanvasHelperService.appendProperties = function (li, properties) {
        li.append('ul').selectAll('li').data(Object.keys(properties)).enter()
            .append('li').html(function (key) {
            if (typeof properties[key] !== 'number') {
                return '  ' + key + ' = "' + properties[key] + '"';
            }
            else {
                return '  ' + key + ' = ' + properties[key];
            }
        });
    };
    ;
    GraphCanvasHelperService.reOrderGraph = function (graph) {
        GraphCanvasHelperService_1.restoreLinks(graph);
        var newGrouping = [];
        var toInsert = graph.groups.filter(function (g) {
            return g.myParent === null;
        });
        while (toInsert.length > 0) {
            var g = toInsert.pop();
            if (g.groups) {
                toInsert = toInsert.concat(g.groups);
            }
            var parentIndex = void 0;
            if (g.myParent === null) {
                parentIndex = -1;
            }
            else {
                parentIndex = newGrouping.indexOf(g.myParent);
            }
            newGrouping.splice(parentIndex + 1, 0, g);
        }
        graph.groups = newGrouping;
        // Regroup Elements
        graph.elements.sort(function (a, b) {
            var sameGroup = graph.groups.indexOf(a.myParent) - graph.groups.indexOf(b.myParent);
            if (!sameGroup) {
                return sameGroup;
            }
            else {
                return a.id - b.id;
            }
        });
        GraphCanvasHelperService_1.reIndexGroups(graph);
    };
    GraphCanvasHelperService.restoreLinks = function (graph) {
        // Restore Links instead of numbers
        graph.groups.forEach(function (g) {
            if (typeof g.groups !== 'undefined') {
                g.groups.forEach(function (l, i) {
                    if (typeof l === 'number') {
                        g.groups[i] = graph.groups[l];
                    }
                });
            }
            g.leaves.forEach(function (l, i) {
                if (typeof l === 'number') {
                    g.leaves[i] = graph.elements[l];
                }
            });
        });
    };
    GraphCanvasHelperService.reIndexGroups = function (graph) {
        graph.groups.forEach(function (g) {
            if (typeof g.groups !== 'undefined') {
                g.groups.forEach(function (l, i) {
                    if (typeof l !== 'number') {
                        g.groups[i] = graph.groups.indexOf(l);
                    }
                });
            }
            g.leaves.forEach(function (l, i) {
                if (typeof l !== 'number') {
                    g.leaves[i] = graph.elements.indexOf(l);
                }
            });
        });
    };
    GraphCanvasHelperService.createDerivedLink = function (l, newSource, newTarget) {
        var hybrid = {
            source: newSource,
            target: newTarget,
            originalSourceTypes: [l.source.type],
            originalTargetTypes: [l.target.type],
            forwardLinks: {},
            backwardLinks: {}
        };
        for (var fKey in l.forwardLinks) {
            if (l.forwardLinks.hasOwnProperty(fKey)) {
                hybrid.forwardLinks['*' + fKey] = l.forwardLinks[fKey];
            }
        }
        for (var bKey in l.backwardLinks) {
            if (l.backwardLinks.hasOwnProperty(bKey)) {
                hybrid.backwardLinks['*' + bKey] = l.backwardLinks[bKey];
            }
        }
        return hybrid;
    };
    GraphCanvasHelperService.prototype.createGroupGraphInitial = function (type, childLinkTypes, crossLinkTypes, graph) {
        var _this = this;
        this.graphTreeService.reset();
        var result = new __WEBPACK_IMPORTED_MODULE_3__graph_data__["a" /* GraphData */]();
        this.graphTreeService.setTopType(type);
        // Check if Element has Children?
        graph.elements.forEach(function (e) {
            var parent = _this.findParent(e, childLinkTypes, graph);
            var element = {
                id: e.id,
                name: e.name,
                type: e.type,
                attributes: e.attributes,
                inHierarchyGraph: true
            };
            _this.graphTreeService.push(element, parent);
        });
        this.graphTreeService.createTree();
        var treeElements = this.graphTreeService.getTree().children.filter(function (e) {
            return e.element.type === type;
        });
        treeElements.forEach(function (e) {
            e.element.myParent = null;
            e.element.isGroup = (e.children.length > 0);
        });
        result.elements = treeElements.map(function (e) {
            return e.element;
        });
        result.links = this.createGraphLinks(result.elements, crossLinkTypes, graph);
        this.mapLinks(result);
        return result;
    };
    /*
     Calculates the links between elements (and all of their children)
     @elements: all elements whose links are calculated
     @crossLinkTypes Linktypes which are included
     @graph: the underlying graph containing all elements, subelements and links
     */
    GraphCanvasHelperService.prototype.createGraphLinks = function (elements, crossLinkTypes, graph) {
        var _this = this;
        var topLinks = [];
        var derivedLinks = [];
        var hybridLinks = [];
        var linkSet = graph.links.filter(function (l) {
            if (typeof crossLinkTypes !== 'undefined' && crossLinkTypes != null && crossLinkTypes.length > 0) {
                for (var _i = 0, crossLinkTypes_1 = crossLinkTypes; _i < crossLinkTypes_1.length; _i++) {
                    var linkType = crossLinkTypes_1[_i];
                    if (l.forwardLinks[linkType] || l.backwardLinks[linkType]) {
                        return true;
                    }
                }
            }
            return false;
        }).map(function (l) {
            return {
                backwardLinks: l.backwardLinks,
                forwardLinks: l.forwardLinks,
                source: l.source,
                target: l.target
            };
        });
        linkSet.forEach(function (l) {
            var source = elements.find(function (e) {
                return __WEBPACK_IMPORTED_MODULE_2__graph_helper_service__["a" /* GraphHelperService */].elementEquals(e, l.source);
            });
            var target = elements.find(function (e) {
                return __WEBPACK_IMPORTED_MODULE_2__graph_helper_service__["a" /* GraphHelperService */].elementEquals(e, l.target);
            });
            if ((typeof source !== 'undefined') && (typeof target !== 'undefined')) {
                topLinks.push(l);
            }
            else {
                if (typeof source === 'undefined') {
                    var sources_1 = _this.graphTreeService.getAncestors(l.source);
                    if (typeof sources_1 === 'undefined') {
                        return;
                    }
                    source = elements.find(function (e) {
                        for (var i = 0; i < sources_1.length; i++) {
                            if (__WEBPACK_IMPORTED_MODULE_2__graph_helper_service__["a" /* GraphHelperService */].elementEquals(e, sources_1[i])) {
                                return true;
                            }
                        }
                        return false;
                    });
                }
                if (typeof target === 'undefined') {
                    var targets_1 = _this.graphTreeService.getAncestors(l.target);
                    if (typeof targets_1 === 'undefined') {
                        return;
                    }
                    target = elements.find(function (e) {
                        for (var i = 0; i < targets_1.length; i++) {
                            if (__WEBPACK_IMPORTED_MODULE_2__graph_helper_service__["a" /* GraphHelperService */].elementEquals(e, targets_1[i])) {
                                return true;
                            }
                        }
                        return false;
                    });
                }
                if (!__WEBPACK_IMPORTED_MODULE_2__graph_helper_service__["a" /* GraphHelperService */].isEmpty(source) && !__WEBPACK_IMPORTED_MODULE_2__graph_helper_service__["a" /* GraphHelperService */].isEmpty(target)) {
                    hybridLinks.push(l);
                    if (source !== target) {
                        var derivedLink = GraphCanvasHelperService_1.createDerivedLink(l, source, target);
                        derivedLinks.push(derivedLink);
                    }
                }
            }
        });
        var allLinks = topLinks.concat(derivedLinks);
        return this.mergeLinks(allLinks);
    };
    /*
     Maps the source und target objects of the links to the right copies
     */
    GraphCanvasHelperService.prototype.mapLinks = function (graph) {
        graph.links.forEach(function (l) {
            graph.elements.find(function (e) {
                return __WEBPACK_IMPORTED_MODULE_2__graph_helper_service__["a" /* GraphHelperService */].elementEquals(l.source, e);
            });
            graph.elements.find(function (e) {
                return __WEBPACK_IMPORTED_MODULE_2__graph_helper_service__["a" /* GraphHelperService */].elementEquals(l.target, e);
            });
        });
    };
    /*
     return parent element of node, if it has none, returns null
     */
    GraphCanvasHelperService.prototype.findParent = function (node, childLinkTypes, graph) {
        var temp = graph.links.find(function (l) {
            if (l.target !== node) {
                return false;
            }
            if (typeof childLinkTypes !== 'undefined' && childLinkTypes != null && childLinkTypes.length > 0) {
                for (var _i = 0, childLinkTypes_1 = childLinkTypes; _i < childLinkTypes_1.length; _i++) {
                    var linkType = childLinkTypes_1[_i];
                    if (l.forwardLinks[linkType]) {
                        return true;
                    }
                }
            }
            return false;
        });
        if (typeof temp !== 'undefined') {
            return temp.source;
        }
        else {
            return null;
        }
    };
    GraphCanvasHelperService.prototype.mergeLinks = function (links) {
        var _this = this;
        var result = [];
        links.forEach(function (l) {
            _this.addToGroupLinks(result, l);
        });
        return result;
    };
    GraphCanvasHelperService.prototype.addToGroupLinks = function (groupLinks, link) {
        var groupLink = groupLinks.find(function (l) {
            return __WEBPACK_IMPORTED_MODULE_2__graph_helper_service__["a" /* GraphHelperService */].elementEquals(l.source, link.source) && __WEBPACK_IMPORTED_MODULE_2__graph_helper_service__["a" /* GraphHelperService */].elementEquals(l.target, link.target);
        });
        if (groupLink) {
            this.addToGroupLink(groupLink, link, true);
        }
        else {
            groupLink = groupLinks.find(function (l) {
                return __WEBPACK_IMPORTED_MODULE_2__graph_helper_service__["a" /* GraphHelperService */].elementEquals(l.source, link.target) && __WEBPACK_IMPORTED_MODULE_2__graph_helper_service__["a" /* GraphHelperService */].elementEquals(l.target, link.source);
            });
            if (groupLink) {
                this.addToGroupLink(groupLink, link, false);
            }
            else {
                groupLinks.push(link);
            }
        }
    };
    GraphCanvasHelperService.prototype.addToGroupLink = function (groupLink, link, sameDirection) {
        var links = [link.forwardLinks, link.backwardLinks];
        var groupLinks;
        if (sameDirection) {
            groupLinks = [groupLink.forwardLinks, groupLink.backwardLinks];
        }
        else {
            groupLinks = [groupLink.backwardLinks, groupLink.forwardLinks];
        }
        for (var i = 0; i < 2; ++i) {
            for (var key in links[i]) {
                if (links[i].hasOwnProperty(key)) {
                    if (groupLinks[i][key]) {
                        groupLinks[i][key] += links[i][key];
                    }
                    else {
                        groupLinks[i][key] = links[i][key];
                    }
                }
            }
        }
        if (typeof groupLink.originalSourceTypes === 'undefined') {
            groupLink.originalSourceTypes = [groupLink.source.type];
            groupLink.originalTargetTypes = [groupLink.target.type];
        }
        if (typeof link.originalSourceTypes === 'undefined') {
            link.originalSourceTypes = [link.source.type];
            link.originalTargetTypes = [link.target.type];
        }
        groupLink.originalSourceTypes =
            this.mergeArraysWithoutDuplicates(groupLink.originalSourceTypes, link.originalSourceTypes);
        groupLink.originalTargetTypes =
            this.mergeArraysWithoutDuplicates(groupLink.originalTargetTypes, link.originalTargetTypes);
    };
    GraphCanvasHelperService.prototype.mergeArraysWithoutDuplicates = function (source, target) {
        var result = source;
        target.forEach(function (targetElement) {
            var contained = false;
            source.forEach(function (sourceElement) {
                if (targetElement === sourceElement) {
                    contained = true;
                }
            });
            if (!contained) {
                result.push(targetElement);
            }
        });
        return result;
    };
    GraphCanvasHelperService.prototype.appendLinkDescription = function (list, links, link) {
        var _this = this;
        var li = list.append('li');
        if (link.originalSourceTypes) {
            link.originalSourceTypes.forEach(function (type) {
                li.append('span').style('background', _this.graphCommunicationService.nodeTypes[type]).text(type);
            });
        }
        else {
            var source = link.source;
            li.append('span').style('background', this.graphCommunicationService.nodeTypes[source.type]).text(__WEBPACK_IMPORTED_MODULE_2__graph_helper_service__["a" /* GraphHelperService */].getNodeName(source));
        }
        li.append('span').text('➝');
        if (link.originalTargetTypes) {
            link.originalTargetTypes.forEach(function (type) {
                li.append('span').style('background', _this.graphCommunicationService.nodeTypes[type]).text(type);
            });
        }
        else {
            var target = link.target;
            li.append('span').style('background', this.graphCommunicationService.nodeTypes[target.type]).text(__WEBPACK_IMPORTED_MODULE_2__graph_helper_service__["a" /* GraphHelperService */].getNodeName(target));
        }
        GraphCanvasHelperService_1.appendProperties(li, links);
    };
    ;
    return GraphCanvasHelperService;
}());
GraphCanvasHelperService = GraphCanvasHelperService_1 = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Injectable"])(),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__graph_tree_service__["a" /* GraphTreeService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__graph_tree_service__["a" /* GraphTreeService */]) === "function" && _a || Object, typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_4__graph_communication_service__["a" /* GraphCommunicationService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_4__graph_communication_service__["a" /* GraphCommunicationService */]) === "function" && _b || Object])
], GraphCanvasHelperService);

var GraphCanvasHelperService_1, _a, _b;
//# sourceMappingURL=graph-canvas-helper.service.js.map

/***/ }),

/***/ "../../../../../src/app/graph/helper/graph-data.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return GraphData; });
var GraphData = (function () {
    function GraphData(json) {
        if (json) {
            this._elements = json['elements'];
            this._links = json['links'];
        }
        else {
            this._elements = [];
            this._links = [];
        }
        this._groups = [];
    }
    Object.defineProperty(GraphData.prototype, "elements", {
        get: function () {
            return this._elements;
        },
        set: function (elements) {
            this._elements = elements;
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(GraphData.prototype, "links", {
        get: function () {
            return this._links;
        },
        set: function (links) {
            this._links = links;
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(GraphData.prototype, "groups", {
        get: function () {
            return this._groups;
        },
        set: function (groups) {
            this._groups = groups;
        },
        enumerable: true,
        configurable: true
    });
    return GraphData;
}());

//# sourceMappingURL=graph-data.js.map

/***/ }),

/***/ "../../../../../src/app/graph/helper/graph-helper.service.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return GraphHelperService; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};

var GraphHelperService = GraphHelperService_1 = (function () {
    function GraphHelperService() {
    }
    GraphHelperService.elementEquals = function (a, b) {
        if (a === null || b === null) {
            return a === b;
        }
        if (typeof a !== 'object' || typeof b !== 'object') {
            return false;
        }
        if (!a.hasOwnProperty('id') || !b.hasOwnProperty('id')) {
            return false;
        }
        return a.id === b.id;
    };
    GraphHelperService.preProcessGraph = function (graph) {
        if (graph.links.length > 0 && graph.links[0].name === 'MergedLink') {
            graph.links = graph.links.map(function (l) {
                return {
                    source: graph.elements[l.source - 1],
                    target: graph.elements[l.target - 1],
                    forwardLinks: l.forwardLinks,
                    backwardLinks: l.backwardLinks
                };
            });
        }
        else {
            graph.links = GraphHelperService_1.preProcessData(graph.elements, graph.links);
        }
        return graph;
    };
    /*
    Combine links
     */
    GraphHelperService.preProcessData = function (elements, links) {
        var groupedLinks = [];
        links.forEach(function (link) {
            var s = elements[link.source - 1];
            var t = elements[link.target - 1];
            if (!groupedLinks.some(function (l) {
                if (l.source === s && l.target === t) {
                    if (l.forwardLinks[link.name]) {
                        ++l.forwardLinks[link.name];
                    }
                    else {
                        l.forwardLinks[link.name] = 1;
                    }
                    return true;
                }
                else if (l.source === t && l.target === s) {
                    if (l.backwardLinks[link.name]) {
                        ++l.backwardLinks[link.name];
                    }
                    else {
                        l.backwardLinks[link.name] = 1;
                    }
                    return true;
                }
            })) {
                var groupedLink = {
                    source: s,
                    target: t,
                    forwardLinks: {},
                    backwardLinks: {}
                };
                groupedLink.forwardLinks[link.name] = 1;
                groupedLinks.push(groupedLink);
            }
        });
        return groupedLinks;
    };
    GraphHelperService.getNodeName = function (e) {
        return e.attributes.simpleName || e.attributes.name;
    };
    GraphHelperService.isEmpty = function (o) {
        for (var key in o) {
            if (o.hasOwnProperty(key)) {
                return false;
            }
        }
        return true;
    };
    return GraphHelperService;
}());
GraphHelperService = GraphHelperService_1 = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Injectable"])(),
    __metadata("design:paramtypes", [])
], GraphHelperService);

var GraphHelperService_1;
//# sourceMappingURL=graph-helper.service.js.map

/***/ }),

/***/ "../../../../../src/app/graph/helper/graph-tree.service.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__graph_helper_service__ = __webpack_require__("../../../../../src/app/graph/helper/graph-helper.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return GraphTreeService; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};


var GraphTreeService = GraphTreeService_1 = (function () {
    function GraphTreeService() {
        this.tree = { element: null, children: [], parent: null };
        this.topTypeTree = { element: null, children: [], parent: null };
        this.allElementsInTree = [this.tree];
        this.unsorted = [];
        this.map = [];
    }
    GraphTreeService.getSubTreeOfElement = function (element, searchTree) {
        if (element === null) {
            return null;
        }
        return GraphTreeService_1.getSubTreeOfElementWithId(element.id, searchTree);
    };
    GraphTreeService.getSubTreeOfElementWithId = function (elementId, searchTree) {
        var list = [searchTree];
        while (list.length > 0) {
            var i = list[0];
            if (i.element !== null && i.element.id === elementId) {
                return i;
            }
            else {
                list.splice(list.indexOf(i), 1);
                if (typeof i.children !== 'undefined') {
                    list = list.concat(i.children);
                }
            }
        }
        return null;
    };
    GraphTreeService.prototype.createTree = function () {
        var _this = this;
        var rootElements = this.allElementsInTree.filter(function (e) {
            return e.parent === null;
        });
        rootElements.shift();
        this.tree.children = rootElements;
        this.topTypeTree.children = rootElements.filter(function (e) {
            return e.element.type === _this.topType;
        });
        this.tree.children.forEach(function (c) {
            if (c.element.type === _this.topType) {
                _this.createSubMap(c, []);
            }
        });
    };
    GraphTreeService.prototype.createSubMap = function (subTree, ancestors) {
        var _this = this;
        var mapElement = { element: subTree.element, ancestors: ancestors };
        this.map.push(mapElement);
        subTree.children.forEach(function (c) {
            _this.createSubMap(c, ancestors.concat(subTree.element));
        });
    };
    GraphTreeService.prototype.push = function (element, parent) {
        var _this = this;
        var newElement = { element: element, children: [], parent: parent };
        // Check if new Element is Parent of unsorted elements
        this.unsorted.forEach(function (unEl) {
            if (__WEBPACK_IMPORTED_MODULE_1__graph_helper_service__["a" /* GraphHelperService */].elementEquals(unEl.parent, newElement.element)) {
                // if (unEl.parent === newElement.element) {
                unEl.parent = newElement.element;
                newElement.children.push(unEl);
                _this.unsorted.splice(_this.unsorted.indexOf(newElement.element), 1);
            }
        });
        // Push to FlatTree
        var i = this.allElementsInTree.find(function (e) {
            return __WEBPACK_IMPORTED_MODULE_1__graph_helper_service__["a" /* GraphHelperService */].elementEquals(parent, e.element);
        });
        if (typeof i !== 'undefined') {
            this.allElementsInTree.push(newElement);
            i.children.push(newElement);
            return;
        }
        this.unsorted.push(newElement);
    };
    GraphTreeService.prototype.contains = function (element) {
        return (GraphTreeService_1.getSubTreeOfElement(element, this.tree) !== null);
    };
    /* public isDescendant(element, ancestor) {
      const mapElement = this.map.find(e => {
        return GraphHelperService.elementEquals(e.element, element);
      });
      if (typeof mapElement === 'undefined') {
        return false;
      }
      return mapElement.ancestors.some(a => {
        return GraphHelperService.elementEquals(a, ancestor);
      });
  
    }*/
    GraphTreeService.prototype.getAncestors = function (element) {
        return this.getInnerAncestors(element, this.topTypeTree);
    };
    GraphTreeService.prototype.getInnerAncestors = function (element, tree) {
        // I am the subroot
        if (__WEBPACK_IMPORTED_MODULE_1__graph_helper_service__["a" /* GraphHelperService */].elementEquals(element, tree.element)) {
            return [tree.element];
        }
        if (typeof tree.children === 'undefined' || tree.children.length === 0) {
            return undefined;
        }
        // Search subtrees;
        for (var i = 0; i < tree.children.length; i++) {
            // tree.children.forEach(c => {
            var ancestors = this.getInnerAncestors(element, tree.children[i]);
            if (typeof ancestors !== 'undefined') {
                ancestors.push(tree.element);
                return ancestors;
            }
        }
    };
    GraphTreeService.prototype.getSubTree = function (elementId) {
        return GraphTreeService_1.getSubTreeOfElementWithId(elementId, this.tree);
    };
    GraphTreeService.prototype.setTopType = function (type) {
        this.topType = type;
    };
    GraphTreeService.prototype.reset = function () {
        this.tree = { element: null, children: [], parent: null };
        this.allElementsInTree = [this.tree];
        this.map = [];
        this.unsorted = [];
        this.topType = null;
    };
    GraphTreeService.prototype.getTree = function () {
        return this.tree;
    };
    return GraphTreeService;
}());
GraphTreeService = GraphTreeService_1 = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Injectable"])(),
    __metadata("design:paramtypes", [])
], GraphTreeService);

var GraphTreeService_1;
//# sourceMappingURL=graph-tree.service.js.map

/***/ }),

/***/ "../../../../../src/app/graph/helper/tab.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return Tab; });
var Tab = (function () {
    function Tab(name, content, active, hierarchy) {
        if (active === void 0) { active = true; }
        if (hierarchy === void 0) { hierarchy = false; }
        this.name = name;
        this.content = content;
        this.active = active;
        this.hierarchy = hierarchy;
    }
    return Tab;
}());

//# sourceMappingURL=tab.js.map

/***/ }),

/***/ "../../../../../src/app/helper/description.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return Description; });
/**
 * Created by johannes on 24.07.17.
 */
var Description = (function () {
    function Description(name, description) {
        this.name = name;
        this.description = description;
    }
    return Description;
}());

//# sourceMappingURL=description.js.map

/***/ }),

/***/ "../../../../../src/app/helper/globalProperties.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return GlobalProperties; });
/**
 * Created by johannes on 04.08.17.
 */
var GlobalProperties = (function () {
    function GlobalProperties(defaultProject, neo4jServer) {
        this.defaultProject = defaultProject;
        this.neo4jServer = neo4jServer;
    }
    GlobalProperties.convertJSONToObject = function (json) {
        var defaultProject = '';
        var neo4j = '/neo4j';
        if (json['defaultProject'] && typeof json['defaultProject'] === 'string') {
            defaultProject = json['defaultProject'];
        }
        if (json['neo4jServer']) {
            neo4j = json['neo4jServer'];
        }
        return new GlobalProperties(defaultProject, neo4j);
    };
    return GlobalProperties;
}());

//# sourceMappingURL=globalProperties.js.map

/***/ }),

/***/ "../../../../../src/app/helper/graphProperties.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return GraphProperties; });
/**
 * Created by johannes on 24.07.17.
 */
var GraphProperties = (function () {
    function GraphProperties(groupItemType, subGroupItemType, groupChildLinkTypes, groupCrossLinkTypes, tabs) {
        this.groupItemType = groupItemType;
        this.subGroupItemType = subGroupItemType;
        this.groupChildLinkTypes = groupChildLinkTypes;
        this.groupCrossLinkTypes = groupCrossLinkTypes;
        this.tabs = tabs;
    }
    return GraphProperties;
}());

//# sourceMappingURL=graphProperties.js.map

/***/ }),

/***/ "../../../../../src/app/helper/graphTab.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return GraphTab; });
/**
 * Created by johannes on 25.07.17.
 */
var GraphTab = (function () {
    function GraphTab(name, rootType, linkList, flatTypes) {
        this.name = name;
        this.rootType = rootType;
        this.linkList = linkList;
        this.flatTypes = flatTypes;
    }
    return GraphTab;
}());

//# sourceMappingURL=graphTab.js.map

/***/ }),

/***/ "../../../../../src/app/helper/httpService.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_http__ = __webpack_require__("../../../http/@angular/http.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1_rxjs_Rx__ = __webpack_require__("../../../../rxjs/Rx.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1_rxjs_Rx___default = __webpack_require__.n(__WEBPACK_IMPORTED_MODULE_1_rxjs_Rx__);
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return HttpService; });


/**
 * Created by johannes on 27.03.17.
 */
var HttpService = (function () {
    function HttpService() {
    }
    HttpService.extractData = function (res) {
        var body = res.json();
        return body || {};
    };
    HttpService.handleError = function (error) {
        // taken from AngularIO Doc
        // In a real world app, you might use a remote logging infrastructure
        var errMsg;
        if (error instanceof __WEBPACK_IMPORTED_MODULE_0__angular_http__["e" /* Response */]) {
            var body = error.json() || '';
            var err = body.error || JSON.stringify(body);
            errMsg = error.status + " - " + (error.statusText || '') + " " + err;
        }
        else {
            errMsg = error.message ? error.message : error.toString();
        }
        console.error(errMsg);
        return __WEBPACK_IMPORTED_MODULE_1_rxjs_Rx__["Observable"].throw(errMsg);
    };
    HttpService.handleErrorWith404Acceptance = function (error) {
        if (error.status === 404) {
            return __WEBPACK_IMPORTED_MODULE_1_rxjs_Rx__["Observable"].of(null);
        }
        return HttpService.handleError(error);
    };
    return HttpService;
}());

//# sourceMappingURL=httpService.js.map

/***/ }),

/***/ "../../../../../src/app/helper/mainProperties.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return MainProperties; });
/**
 * Created by johannes on 24.07.17.
 */
var MainProperties = (function () {
    function MainProperties(defaultFile, showAllDropdownElements, dropdownElements) {
        if (showAllDropdownElements === void 0) { showAllDropdownElements = true; }
        this.defaultFile = defaultFile;
        this.showAllDropdownElements = showAllDropdownElements;
        this.dropdownElements = dropdownElements;
    }
    return MainProperties;
}());

//# sourceMappingURL=mainProperties.js.map

/***/ }),

/***/ "../../../../../src/app/helper/metricGraphicalProp.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return MetricGraphicalProp; });
/**
 * Created by johannes on 24.07.17.
 */
var MetricGraphicalProp = (function () {
    function MetricGraphicalProp(name, graphic, defaultGraphic, showMultipleMetric, relMetric, relMetricX, relMetricY, coloring, coloringMin, coloringMax) {
        if (graphic === void 0) { graphic = []; }
        if (defaultGraphic === void 0) { defaultGraphic = ''; }
        if (showMultipleMetric === void 0) { showMultipleMetric = false; }
        if (coloring === void 0) { coloring = false; }
        this.name = name;
        this.graphic = graphic;
        this.defaultGraphic = defaultGraphic;
        this.showMultipleMetrics = showMultipleMetric;
        this.relMetric = relMetric;
        this.relMetricX = relMetricX;
        this.relMetricY = relMetricY;
        this.coloring = coloring;
        this.coloringMin = coloringMin;
        this.coloringMax = coloringMax;
    }
    return MetricGraphicalProp;
}());

//# sourceMappingURL=metricGraphicalProp.js.map

/***/ }),

/***/ "../../../../../src/app/helper/metricProperties.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return MetricProperties; });
/**
 * Created by johannes on 24.07.17.
 */
var MetricProperties = (function () {
    function MetricProperties(graphical, descriptions, subGroups, usedOd) {
        if (graphical === void 0) { graphical = []; }
        if (descriptions === void 0) { descriptions = []; }
        if (subGroups === void 0) { subGroups = []; }
        if (usedOd === void 0) { usedOd = ''; }
        this.graphical = graphical;
        this.descriptions = descriptions;
        this.subGroups = subGroups;
        this.usedOd = usedOd;
    }
    return MetricProperties;
}());

//# sourceMappingURL=metricProperties.js.map

/***/ }),

/***/ "../../../../../src/app/helper/objectdiagram.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return Objectdiagram; });
var Objectdiagram = (function () {
    function Objectdiagram(name, path, jsonPath) {
        this._name = name;
        this._path = path;
        this._jsonPath = jsonPath;
    }
    Object.defineProperty(Objectdiagram.prototype, "name", {
        get: function () {
            return this._name;
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(Objectdiagram.prototype, "path", {
        get: function () {
            return this._path;
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(Objectdiagram.prototype, "jsonPath", {
        get: function () {
            return this._jsonPath;
        },
        enumerable: true,
        configurable: true
    });
    return Objectdiagram;
}());

//# sourceMappingURL=objectdiagram.js.map

/***/ }),

/***/ "../../../../../src/app/helper/project.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return Project; });
var Project = (function () {
    function Project(name, path) {
        this._name = name;
        this._path = path;
    }
    Object.defineProperty(Project.prototype, "name", {
        get: function () {
            return this._name;
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(Project.prototype, "path", {
        get: function () {
            return this._path;
        },
        enumerable: true,
        configurable: true
    });
    return Project;
}());

//# sourceMappingURL=project.js.map

/***/ }),

/***/ "../../../../../src/app/helper/properties.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__mainProperties__ = __webpack_require__("../../../../../src/app/helper/mainProperties.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__textualProperties__ = __webpack_require__("../../../../../src/app/helper/textualProperties.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__graphProperties__ = __webpack_require__("../../../../../src/app/helper/graphProperties.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__metricProperties__ = __webpack_require__("../../../../../src/app/helper/metricProperties.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__metricGraphicalProp__ = __webpack_require__("../../../../../src/app/helper/metricGraphicalProp.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5__description__ = __webpack_require__("../../../../../src/app/helper/description.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_6__subGroups__ = __webpack_require__("../../../../../src/app/helper/subGroups.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_7__graphTab__ = __webpack_require__("../../../../../src/app/helper/graphTab.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return Properties; });








/**
 * Created by johannes on 24.07.17.
 */
var Properties = (function () {
    function Properties(main, textual, graph, metrics) {
        if (main === void 0) { main = null; }
        if (textual === void 0) { textual = null; }
        if (graph === void 0) { graph = null; }
        if (metrics === void 0) { metrics = null; }
        if (main != null) {
            this.main = main;
        }
        else {
            this.main = new __WEBPACK_IMPORTED_MODULE_0__mainProperties__["a" /* MainProperties */]('');
        }
        if (textual != null) {
            this.textual = textual;
        }
        else {
            this.textual = new __WEBPACK_IMPORTED_MODULE_1__textualProperties__["a" /* TextualProperties */](null);
        }
        if (graph != null) {
            this.graph = graph;
        }
        else {
            this.graph = new __WEBPACK_IMPORTED_MODULE_2__graphProperties__["a" /* GraphProperties */]('', [], [], [], []);
        }
        if (metrics != null) {
            this.metrics = metrics;
        }
        else {
            this.metrics = new __WEBPACK_IMPORTED_MODULE_3__metricProperties__["a" /* MetricProperties */]();
        }
    }
    Properties.convertJSONToObject = function (json) {
        var mainProp = new __WEBPACK_IMPORTED_MODULE_0__mainProperties__["a" /* MainProperties */]('');
        var textProp;
        var metricProp;
        var graphical = [];
        var descriptions = [];
        var subGroups = [];
        var graphProp;
        var graphTabs = [];
        if (json['main']) {
            mainProp = new __WEBPACK_IMPORTED_MODULE_0__mainProperties__["a" /* MainProperties */](json['main'].defaultFile, json['main'].showAllDropdownElements, json['main'].dropdownElements);
        }
        if (json['textual']) {
            textProp = new __WEBPACK_IMPORTED_MODULE_1__textualProperties__["a" /* TextualProperties */](json['textual'].extensions);
        }
        if (json['metrics']) {
            var g = json['metrics'].graphical;
            for (var graphi in g) {
                if (!g.hasOwnProperty(graphi)) {
                    continue;
                }
                var charts = g[graphi].graphic;
                var defaultChart = g[graphi].defaultGraphic;
                if (charts && !defaultChart) {
                    defaultChart = charts[0];
                }
                var element = new __WEBPACK_IMPORTED_MODULE_4__metricGraphicalProp__["a" /* MetricGraphicalProp */](graphi, charts, defaultChart, g[graphi].showMultipleMetrics, g[graphi].relMetric, g[graphi].relMetricX, g[graphi].relMetricY, g[graphi].coloring, g[graphi].coloringMin, g[graphi].coloringMax);
                graphical.push(element);
            }
            var d = json['metrics'].descriptions;
            for (var desc in d) {
                if (!d.hasOwnProperty(desc)) {
                    continue;
                }
                var element = new __WEBPACK_IMPORTED_MODULE_5__description__["a" /* Description */](desc, d[desc]);
                descriptions.push(element);
            }
            if (json['metrics'].hasOwnProperty('subGroups')) {
                for (var _i = 0, _a = json['metrics'].subGroups; _i < _a.length; _i++) {
                    var sub = _a[_i];
                    var element = new __WEBPACK_IMPORTED_MODULE_6__subGroups__["a" /* SubGroups */](sub.metrics, sub.types);
                    subGroups.push(element);
                }
            }
            metricProp = new __WEBPACK_IMPORTED_MODULE_3__metricProperties__["a" /* MetricProperties */](graphical, descriptions, subGroups, json['metrics'].usedOd);
        }
        if (json['graph']) {
            if (json['graph'].hasOwnProperty('tabs')) {
                var t = json['graph'].tabs;
                for (var tab in t) {
                    if (!t.hasOwnProperty(tab)) {
                        continue;
                    }
                    var element = new __WEBPACK_IMPORTED_MODULE_7__graphTab__["a" /* GraphTab */](tab, t[tab].rootType, t[tab].linkList, t[tab].flatTypes);
                    graphTabs.push(element);
                }
            }
            var groupItemType = '';
            var subGroupItemType = [];
            var groupChildLinkTypes = [];
            var groupCrossLinkTypes = [];
            if (json['graph'].hasOwnProperty('hierarchy')) {
                groupItemType = json['graph'].hierarchy.groupItemType;
                subGroupItemType = json['graph'].hierarchy.subGroupItemType;
                groupChildLinkTypes = json['graph'].hierarchy.groupChildLinkTypes;
                groupCrossLinkTypes = json['graph'].hierarchy.groupCrossLinkTypes;
            }
            graphProp = new __WEBPACK_IMPORTED_MODULE_2__graphProperties__["a" /* GraphProperties */](groupItemType, subGroupItemType, groupChildLinkTypes, groupCrossLinkTypes, graphTabs);
        }
        return new Properties(mainProp, textProp, graphProp, metricProp);
    };
    return Properties;
}());

//# sourceMappingURL=properties.js.map

/***/ }),

/***/ "../../../../../src/app/helper/subGroups.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return SubGroups; });
/**
 * Created by johannes on 24.07.17.
 */
var SubGroups = (function () {
    function SubGroups(metrics, types) {
        this.metrics = metrics;
        this.types = types;
    }
    return SubGroups;
}());

//# sourceMappingURL=subGroups.js.map

/***/ }),

/***/ "../../../../../src/app/helper/textualProperties.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return TextualProperties; });
/**
 * Created by johannes on 24.07.17.
 */
var TextualProperties = (function () {
    function TextualProperties(extensions) {
        this.extensions = extensions;
    }
    return TextualProperties;
}());

//# sourceMappingURL=textualProperties.js.map

/***/ }),

/***/ "../../../../../src/app/metric/helper/metric-data-item.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return MetricDataItem; });
var MetricDataItem = (function () {
    function MetricDataItem(name, type, value) {
        this.name = name;
        this.type = type;
        this.value = value;
    }
    return MetricDataItem;
}());

//# sourceMappingURL=metric-data-item.js.map

/***/ }),

/***/ "../../../../../src/app/metric/helper/metric.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return Metric; });
var Metric = (function () {
    function Metric(name, data) {
        this.name = name;
        this.data = data;
    }
    Object.defineProperty(Metric.prototype, "description", {
        get: function () {
            return this._description;
        },
        set: function (description) {
            this._description = description;
        },
        enumerable: true,
        configurable: true
    });
    return Metric;
}());

//# sourceMappingURL=metric.js.map

/***/ }),

/***/ "../../../../../src/app/metric/helper/table.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return Table; });
var Table = (function () {
    function Table(columns, rows, types, metrics, name) {
        this.rows = rows;
        this.columns = columns;
        this.types = types;
        this.metrics = metrics;
        if (name) {
            this.name = name;
        }
        else {
            this.name = metrics.reduce(function (prev, akt) {
                return prev + ', ' + akt.substring(0, 3);
            });
        }
    }
    return Table;
}());

//# sourceMappingURL=table.js.map

/***/ }),

/***/ "../../../../../src/app/metric/metric-charts/metric-charts.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, "", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/metric/metric-charts/metric-charts.component.html":
/***/ (function(module, exports) {

module.exports = "<ngb-carousel #chartCarousel *ngIf=\"displayCharts\"\n[interval]=\"100000000\">\n  <ng-template ngbSlide id=horBarChart *ngIf=\"displayHorBarChart\">\n    <ngx-charts-bar-horizontal-2d\n      [view]=\"view\"\n      [scheme]=\"colorScheme\"\n      [results]=\"multiData\"\n      [legend]=\"true\"\n      [xAxis]=\"true\"\n      [yAxis]=\"true\"\n      (select)=\"onSelect($event)\">\n    </ngx-charts-bar-horizontal-2d>\n  </ng-template>\n\n  <ng-template ngbSlide id=vertBarChart *ngIf=\"displayVertBarChart\">\n    <ngx-charts-bar-vertical\n                             [view]=\"view\"\n                             [scheme]=\"colorScheme\"\n                             [results]=\"singleData\"\n                             [xAxis]=\"true\"\n                             [yAxis]=\"true\"\n                             [legend]=\"true\"\n                             (select)=\"onSelect($event)\">\n    </ngx-charts-bar-vertical>\n  </ng-template>\n\n  <ng-template ngbSlide id=pieChart *ngIf=\"displayPieChart\">\n    <ngx-charts-pie-chart\n      [view]=\"view\"\n      [scheme]=\"colorScheme\"\n      [results]=\"singleData\"\n      [labels]=\"true\"\n      (select)=\"onSelect($event)\">\n    </ngx-charts-pie-chart>\n  </ng-template>\n\n  <ng-template ngbSlide id=scatterChart *ngIf=\"displayScatterChart\">\n    <ngx-charts-bubble-chart\n      [view]=\"view\"\n      [scheme]=\"colorScheme\"\n      [results]=\"scatterData\"\n      [xAxis]=\"true\"\n      [yAxis]=\"true\"\n      [showXAxisLabel]=\"true\"\n      [showYAxisLabel]=\"true\"\n      [xAxisLabel]=\"scatterXLabel\"\n      [yAxisLabel]=\"scatterYLabel\"\n\n      (select)=\"onSelect($event)\"\n    >\n    </ngx-charts-bubble-chart>\n\n  </ng-template>\n\n</ngb-carousel>\n\n\n\n"

/***/ }),

/***/ "../../../../../src/app/metric/metric-charts/metric-charts.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__metric_service__ = __webpack_require__("../../../../../src/app/metric/metric.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2_d3_ng2_service__ = __webpack_require__("../../../../d3-ng2-service/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__ng_bootstrap_ng_bootstrap__ = __webpack_require__("../../../../@ng-bootstrap/ng-bootstrap/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__color_service__ = __webpack_require__("../../../../../src/app/color.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return MetricChartsComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};





var MetricChartsComponent = (function () {
    function MetricChartsComponent(metricService, d3Service, colorService) {
        var _this = this;
        this.metricService = metricService;
        this.d3Service = d3Service;
        this.colorService = colorService;
        this.view = [650, 400];
        this.d3 = d3Service.getD3();
        this.setColorScheme();
        metricService.metrics$.subscribe(function (metrics) {
            if (metrics.length > 0) {
                _this.metrics = metrics;
            }
        });
    }
    Object.defineProperty(MetricChartsComponent.prototype, "metricToShow", {
        get: function () {
            return this._metricToShow;
        },
        set: function (metric) {
            this._metricToShow = metric;
            this.update();
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(MetricChartsComponent.prototype, "onlyHighest", {
        get: function () {
            return this._onlyHighest;
        },
        set: function (onlyHighest) {
            this._onlyHighest = onlyHighest;
            if (this.currentMetric != null) {
                this.calcChartData(this.currentMetric);
            }
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(MetricChartsComponent.prototype, "highestCount", {
        get: function () {
            return this._highestCount;
        },
        set: function (count) {
            this._highestCount = count;
            if (this.currentMetric != null) {
                this.calcChartData(this.currentMetric);
            }
        },
        enumerable: true,
        configurable: true
    });
    Object.defineProperty(MetricChartsComponent.prototype, "properties", {
        get: function () {
            return this._properties;
        },
        set: function (properites) {
            this._properties = properites;
            this.update();
        },
        enumerable: true,
        configurable: true
    });
    MetricChartsComponent.prototype.setColorScheme = function () {
        var domain = this.colorService.getColorScheme();
        this.colorScheme = { domain: domain };
    };
    MetricChartsComponent.prototype.ngOnInit = function () {
    };
    MetricChartsComponent.prototype.onSelect = function (event) {
        console.log(event);
    };
    MetricChartsComponent.prototype.update = function () {
        if (this.metricToShow === 'Metric') {
            this.displayCharts = false;
        }
        if (this.metricToShow !== 'Metric' && this.properties != null) {
            this.updateGraphics();
        }
    };
    MetricChartsComponent.prototype.updateGraphics = function () {
        var _this = this;
        this.currentProperties = this.properties.find(function (p) {
            return p.name === _this.metricToShow;
        });
        if (this.currentProperties == null) {
            console.log('Error: Metric not displayable');
            return;
        }
        var currentMetric = this.metrics.find(function (m) {
            return m.name === _this.metricToShow;
        });
        this.currentMetric = currentMetric;
        if (this.currentProperties.graphic.length > 0) {
            this.calcChartData(currentMetric);
            this.displayVertBarChart = (this.currentProperties.graphic.indexOf('Vertical Bar Chart') !== -1);
            this.displayPieChart = (this.currentProperties.graphic.indexOf('Pie Chart') !== -1);
            this.displayScatterChart = (this.currentProperties.graphic.indexOf('Scatter Chart') !== -1);
            this.displayHorBarChart = (this.currentProperties.graphic.indexOf('Horizontal Bar Chart') !== -1);
            this.displayCharts = true;
            setTimeout(function () {
                switch (_this.currentProperties.defaultGraphic) {
                    case 'Horizontal Bar Chart':
                        _this.chartCarousel.select('horBarChart');
                        break;
                    case 'Vertical Bar Chart':
                        _this.chartCarousel.select('vertBarChart');
                        break;
                    case 'Pie Chart':
                        _this.chartCarousel.select('pieChart');
                        break;
                    case 'Scatter Chart':
                        _this.chartCarousel.select('scatterChart');
                        break;
                    default:
                        _this.displayCharts = false;
                        console.log('Unsupported Default Chart');
                }
            }, 0);
        }
        else {
            this.displayCharts = false;
        }
    };
    MetricChartsComponent.prototype.calcChartData = function (metric) {
        var metricData = metric.data;
        metricData = metricData.sort(function (a, b) {
            return b.value - a.value;
        });
        var count = metricData.length;
        if (this.onlyHighest) {
            count = this.highestCount;
        }
        this.calcSingleData(metricData, count);
        this.calcMultiData(metricData, count, metric.name);
        this.calcScatterData(metricData, count, metric.name);
    };
    MetricChartsComponent.prototype.calcSingleData = function (metricData, count) {
        var result = [];
        var value = {};
        for (var i = 0; i < count; i++) {
            value = { name: metricData[i].name, value: metricData[i].value };
            result.push(value);
        }
        this.singleData = result;
    };
    MetricChartsComponent.prototype.calcMultiData = function (metricData, count, metricName) {
        var _this = this;
        var result = [];
        for (var i = 0; i < count; i++) {
            var series = [];
            series.push({ name: metricName, value: metricData[i].value });
            var element = { name: metricData[i].name, series: series };
            result.push(element);
        }
        if (this.currentProperties.showMultipleMetrics) {
            var secondMetric = this.metrics.find(function (m) {
                return m.name === _this.currentProperties.relMetric;
            });
            if (secondMetric != null) {
                var secondMetricData = secondMetric.data;
                var _loop_1 = function (artifact) {
                    var item = secondMetricData.find(function (d) {
                        return d.name === artifact.name;
                    });
                    var seriesItem = {};
                    if (item) {
                        seriesItem = { name: secondMetric.name, value: item.value };
                    }
                    else {
                        seriesItem = { name: secondMetric.name, value: 0 };
                    }
                    artifact.series.push(seriesItem);
                };
                for (var _i = 0, result_1 = result; _i < result_1.length; _i++) {
                    var artifact = result_1[_i];
                    _loop_1(artifact);
                }
            }
        }
        this.multiData = result;
    };
    MetricChartsComponent.prototype.calcScatterData = function (metricData, count, metricName) {
        var _this = this;
        var series = [];
        for (var i = 0; i < count; i++) {
            var element = { name: metricData[i].name, r: metricData[i].value };
            series.push(element);
        }
        var xMetric = this.metrics.find(function (m) {
            return m.name === _this.currentProperties.relMetricX;
        });
        var yMetric = this.metrics.find(function (m) {
            return m.name === _this.currentProperties.relMetricY;
        });
        if (xMetric != null && yMetric != null) {
            var xMetricData = xMetric.data;
            var yMetricData = yMetric.data;
            this.scatterXLabel = xMetric.name;
            this.scatterYLabel = yMetric.name;
            var _loop_2 = function (element) {
                var xItem = xMetricData.find(function (d) {
                    return d.name === element.name;
                });
                if (xItem) {
                    element.x = xItem.value;
                }
                else {
                    element.x = 0;
                }
                var yItem = yMetricData.find(function (d) {
                    return d.name === element.name;
                });
                if (yItem) {
                    element.y = yItem.value;
                }
                else {
                    element.y = 0;
                }
            };
            for (var _i = 0, series_1 = series; _i < series_1.length; _i++) {
                var element = series_1[_i];
                _loop_2(element);
            }
        }
        var result = [];
        var value = { name: metricName, series: series };
        result.push(value);
        this.scatterData = result;
    };
    return MetricChartsComponent;
}());
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["ViewChild"])(__WEBPACK_IMPORTED_MODULE_3__ng_bootstrap_ng_bootstrap__["d" /* NgbCarousel */]),
    __metadata("design:type", Object)
], MetricChartsComponent.prototype, "chartCarousel", void 0);
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", Object),
    __metadata("design:paramtypes", [Object])
], MetricChartsComponent.prototype, "metricToShow", null);
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", Boolean),
    __metadata("design:paramtypes", [Boolean])
], MetricChartsComponent.prototype, "onlyHighest", null);
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", Number),
    __metadata("design:paramtypes", [Number])
], MetricChartsComponent.prototype, "highestCount", null);
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", Object),
    __metadata("design:paramtypes", [Object])
], MetricChartsComponent.prototype, "properties", null);
MetricChartsComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-metric-charts',
        template: __webpack_require__("../../../../../src/app/metric/metric-charts/metric-charts.component.html"),
        styles: [__webpack_require__("../../../../../src/app/metric/metric-charts/metric-charts.component.css")]
    }),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__metric_service__["a" /* MetricService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__metric_service__["a" /* MetricService */]) === "function" && _a || Object, typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_2_d3_ng2_service__["a" /* D3Service */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_2_d3_ng2_service__["a" /* D3Service */]) === "function" && _b || Object, typeof (_c = typeof __WEBPACK_IMPORTED_MODULE_4__color_service__["a" /* ColorService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_4__color_service__["a" /* ColorService */]) === "function" && _c || Object])
], MetricChartsComponent);

var _a, _b, _c;
//# sourceMappingURL=metric-charts.component.js.map

/***/ }),

/***/ "../../../../../src/app/metric/metric-routing.module.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__angular_router__ = __webpack_require__("../../../router/@angular/router.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__metric_metric_component__ = __webpack_require__("../../../../../src/app/metric/metric/metric.component.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return MetricRoutingModule; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};



var routes = [
    { path: '', component: __WEBPACK_IMPORTED_MODULE_2__metric_metric_component__["a" /* MetricComponent */] }
];
var MetricRoutingModule = (function () {
    function MetricRoutingModule() {
    }
    return MetricRoutingModule;
}());
MetricRoutingModule = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["NgModule"])({
        imports: [__WEBPACK_IMPORTED_MODULE_1__angular_router__["a" /* RouterModule */].forChild(routes)],
        exports: [__WEBPACK_IMPORTED_MODULE_1__angular_router__["a" /* RouterModule */]]
    })
], MetricRoutingModule);

//# sourceMappingURL=metric-routing.module.js.map

/***/ }),

/***/ "../../../../../src/app/metric/metric-table/metric-table.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, "", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/metric/metric-table/metric-table.component.html":
/***/ (function(module, exports) {

module.exports = "<div class=\"col-md-12\">\n  <ngb-tabset (tabChange)=\"subTableChanged($event)\">\n    <ngb-tab *ngFor=\"let subTable of subTables\" id=\"tab-{{subTable.name}}\" title=\"{{subTable.name}}\">\n\n      <ng-template ngbTabContent>\n\n        <h6>Metrics: {{subTable.metrics}}</h6>\n        <h6>Types: {{subTable.types}}</h6>\n\n        <ngx-datatable\n          class=\"material striped\"\n          [rows]=\"subTable.rows\"\n          [limit]=\"20\"\n          [footerHeight]=\"50\"\n          [headerHeight]=\"40\"\n          [scrollbarH]=\"true\"\n        >\n          <ngx-datatable-column *ngFor=\"let myColumn of subTable.columns\"\n                                [name]=\"myColumn.name\"\n                                [prop]=\"myColumn.prop\"\n                                [width]=\"myColumn.defaultWidth\"\n          >\n            <ng-template ngx-datatable-header-template let-sort=\"sortFn\" let-column=\"column\">\n              <span class=\"draggable\" (click)=\"sort()\" ngbTooltip=\"{{myColumn.description}}\" container=\"body\" placement=\"top\">\n                {{column.name}}\n              </span>\n            </ng-template>\n\n\n            <ng-template ngx-datatable-cell-template let-value=\"value\">\n              <div *ngIf=\"mustBeColored(myColumn)\" [ngStyle]=\"{'background-color': getColor(value)}\">{{value}}</div>\n              <div *ngIf=\"!mustBeColored(myColumn)\">{{value}}</div>\n            </ng-template>\n          </ngx-datatable-column>\n        </ngx-datatable>\n\n      </ng-template>\n\n    </ngb-tab>\n  </ngb-tabset>\n</div>\n"

/***/ }),

/***/ "../../../../../src/app/metric/metric-table/metric-table.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__metric_service__ = __webpack_require__("../../../../../src/app/metric/metric.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__selected_project_service__ = __webpack_require__("../../../../../src/app/selected-project.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__helper_table__ = __webpack_require__("../../../../../src/app/metric/helper/table.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__helper_subGroups__ = __webpack_require__("../../../../../src/app/helper/subGroups.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return MetricTableComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};





var MetricTableComponent = (function () {
    function MetricTableComponent(metricService, selectedProjectService) {
        var _this = this;
        this.metricService = metricService;
        this.selectedProjectService = selectedProjectService;
        this.onSubTableChange = new __WEBPACK_IMPORTED_MODULE_0__angular_core__["EventEmitter"]();
        this.standardColumns = [
            {
                name: 'Artifacts',
                prop: 'artifact',
                defaultWidth: 200
            },
            { name: 'Type', prop: 'type', defaultWidth: 150 }
        ];
        this.getColor = function (value) {
            var max = this.coloring.max;
            var min = this.coloring.min;
            var normalizedValue = value - min / (max - min);
            /**
             * between 1 and 2
             * if set to 1 then 0.5 value is more olive than yellow
             * if set to 2 then differences between values are harder to see
             */
            var yellowModifier = 1.7;
            var red = Math.round(yellowModifier * 255 * normalizedValue);
            var green = Math.round(yellowModifier * 255 * (1 - normalizedValue));
            var blue = 0;
            // Max values
            if (red > 255) {
                red = 255;
            }
            if (green > 255) {
                green = 255;
            }
            return 'rgb(' + red + ',' + green + ',' + blue + ')';
        };
        metricService.metrics$.subscribe(function (metrics) {
            if (metrics.length > 0) {
                _this.allMetrics = metrics;
                _this.update();
            }
        });
    }
    MetricTableComponent.prototype.ngOnInit = function () {
    };
    MetricTableComponent.prototype.ngAfterViewInit = function () {
        var metrics = this.metricService.getMetrics().getValue();
        if (metrics.length > 0) {
            this.allMetrics = metrics;
            this.update();
        }
    };
    MetricTableComponent.prototype.update = function () {
        // Get Properties also updates SubTables
        this.getProperties();
    };
    MetricTableComponent.prototype.getProperties = function () {
        var _this = this;
        return new Promise(function (resolve) {
            _this.selectedProjectService.getProperties().subscribe(function (properties) {
                var subGroups = properties.metrics.subGroups;
                var descriptions = properties.metrics.descriptions;
                _this.makeSubTables(subGroups, descriptions);
                resolve();
            });
        });
    };
    MetricTableComponent.prototype.makeSubTables = function (subGroups, descriptions) {
        // If no subGroups show all Metrics
        if (subGroups.length === 0) {
            var metrics_1 = [];
            var types_1 = [];
            this.allMetrics.forEach(function (metric) {
                if (metrics_1.indexOf(metric.name) === -1) {
                    metrics_1.push(metric.name);
                }
                metric.data.forEach(function (item) {
                    if (types_1.indexOf(item.type) === -1) {
                        types_1.push(item.type);
                    }
                });
            });
            subGroups = [new __WEBPACK_IMPORTED_MODULE_4__helper_subGroups__["a" /* SubGroups */](metrics_1, types_1)];
        }
        this.subTables = [];
        for (var _i = 0, subGroups_1 = subGroups; _i < subGroups_1.length; _i++) {
            var subGroup = subGroups_1[_i];
            var metrics = subGroup['metrics'];
            var types = subGroup['types'];
            // Slice(0) is Cloning
            var columns = this.standardColumns.slice(0);
            var rows = [];
            var _loop_1 = function (metric) {
                var name = metric.toLocaleLowerCase().replace(' ', '');
                var description = descriptions.find(function (d) {
                    return d.name === metric;
                });
                var column = { name: metric, prop: name, defaultWidth: 100 };
                if (description) {
                    column['description'] = description.description;
                }
                columns.push(column);
                var metricData = this_1.allMetrics.find(function (m) {
                    return metric === m.name;
                })
                    .data;
                var _loop_2 = function (metricDataItem) {
                    var row = rows.find(function (r) {
                        return r['artifact'] === metricDataItem.name;
                    });
                    // No triple equals
                    if (row == null) {
                        row = { artifact: metricDataItem.name, type: metricDataItem.type };
                        for (var _i = 0, metrics_2 = metrics; _i < metrics_2.length; _i++) {
                            var innerMetric = metrics_2[_i];
                            var innerName = innerMetric.toLocaleLowerCase().replace(' ', '');
                            row[innerName] = '-';
                        }
                        rows.push(row);
                    }
                    var value = metricDataItem.value;
                    if (typeof value === 'number') {
                        value = Math.round(value * 10000) / 10000;
                    }
                    row[name] = value;
                };
                for (var _i = 0, metricData_1 = metricData; _i < metricData_1.length; _i++) {
                    var metricDataItem = metricData_1[_i];
                    _loop_2(metricDataItem);
                }
            };
            var this_1 = this;
            for (var _a = 0, metrics_3 = metrics; _a < metrics_3.length; _a++) {
                var metric = metrics_3[_a];
                _loop_1(metric);
            }
            var newTable = new __WEBPACK_IMPORTED_MODULE_3__helper_table__["a" /* Table */](columns, rows, types, metrics);
            this.subTables.push(newTable);
        }
        this.onSubTableChange.emit(this.subTables[0]);
    };
    MetricTableComponent.prototype.subTableChanged = function ($event) {
        var nextTableName = $event.nextId.substr(4);
        var nextTable = this.subTables.find(function (e) {
            return nextTableName === e.name;
        });
        this.onSubTableChange.emit(nextTable);
    };
    MetricTableComponent.prototype.mustBeColored = function (column) {
        return this.coloring.active && column.name === this.selectedMetric;
    };
    return MetricTableComponent;
}());
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", String)
], MetricTableComponent.prototype, "selectedMetric", void 0);
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", Object)
], MetricTableComponent.prototype, "coloring", void 0);
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Output"])(),
    __metadata("design:type", Object)
], MetricTableComponent.prototype, "onSubTableChange", void 0);
MetricTableComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-metric-table',
        template: __webpack_require__("../../../../../src/app/metric/metric-table/metric-table.component.html"),
        styles: [__webpack_require__("../../../../../src/app/metric/metric-table/metric-table.component.css")],
        encapsulation: __WEBPACK_IMPORTED_MODULE_0__angular_core__["ViewEncapsulation"].None,
    }),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__metric_service__["a" /* MetricService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__metric_service__["a" /* MetricService */]) === "function" && _a || Object, typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_2__selected_project_service__["a" /* SelectedProjectService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_2__selected_project_service__["a" /* SelectedProjectService */]) === "function" && _b || Object])
], MetricTableComponent);

var _a, _b;
//# sourceMappingURL=metric-table.component.js.map

/***/ }),

/***/ "../../../../../src/app/metric/metric.module.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
Object.defineProperty(__webpack_exports__, "__esModule", { value: true });
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__angular_common__ = __webpack_require__("../../../common/@angular/common.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__metric_metric_component__ = __webpack_require__("../../../../../src/app/metric/metric/metric.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__metric_routing_module__ = __webpack_require__("../../../../../src/app/metric/metric-routing.module.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__metric_table_metric_table_component__ = __webpack_require__("../../../../../src/app/metric/metric-table/metric-table.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5__metric_charts_metric_charts_component__ = __webpack_require__("../../../../../src/app/metric/metric-charts/metric-charts.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_6__metric_service__ = __webpack_require__("../../../../../src/app/metric/metric.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_7__selected_project_service__ = __webpack_require__("../../../../../src/app/selected-project.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_8__ng_bootstrap_ng_bootstrap__ = __webpack_require__("../../../../@ng-bootstrap/ng-bootstrap/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_9__angular_forms__ = __webpack_require__("../../../forms/@angular/forms.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_10__selected_od_service__ = __webpack_require__("../../../../../src/app/selected-od.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_11__swimlane_ngx_charts__ = __webpack_require__("../../../../@swimlane/ngx-charts/release/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_11__swimlane_ngx_charts___default = __webpack_require__.n(__WEBPACK_IMPORTED_MODULE_11__swimlane_ngx_charts__);
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_12_d3_ng2_service__ = __webpack_require__("../../../../d3-ng2-service/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_13__swimlane_ngx_datatable__ = __webpack_require__("../../../../@swimlane/ngx-datatable/release/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_13__swimlane_ngx_datatable___default = __webpack_require__.n(__WEBPACK_IMPORTED_MODULE_13__swimlane_ngx_datatable__);
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "MetricModule", function() { return MetricModule; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};














var MetricModule = MetricModule_1 = (function () {
    function MetricModule() {
    }
    MetricModule.forRoot = function () {
        return {
            ngModule: MetricModule_1,
            providers: [__WEBPACK_IMPORTED_MODULE_7__selected_project_service__["a" /* SelectedProjectService */], __WEBPACK_IMPORTED_MODULE_10__selected_od_service__["a" /* SelectedOdService */], __WEBPACK_IMPORTED_MODULE_12_d3_ng2_service__["a" /* D3Service */]]
        };
    };
    return MetricModule;
}());
MetricModule = MetricModule_1 = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["NgModule"])({
        imports: [
            __WEBPACK_IMPORTED_MODULE_1__angular_common__["CommonModule"],
            __WEBPACK_IMPORTED_MODULE_3__metric_routing_module__["a" /* MetricRoutingModule */],
            __WEBPACK_IMPORTED_MODULE_8__ng_bootstrap_ng_bootstrap__["a" /* NgbModule */],
            __WEBPACK_IMPORTED_MODULE_9__angular_forms__["a" /* FormsModule */],
            __WEBPACK_IMPORTED_MODULE_11__swimlane_ngx_charts__["NgxChartsModule"],
            __WEBPACK_IMPORTED_MODULE_13__swimlane_ngx_datatable__["NgxDatatableModule"]
        ],
        declarations: [__WEBPACK_IMPORTED_MODULE_2__metric_metric_component__["a" /* MetricComponent */], __WEBPACK_IMPORTED_MODULE_4__metric_table_metric_table_component__["a" /* MetricTableComponent */], __WEBPACK_IMPORTED_MODULE_5__metric_charts_metric_charts_component__["a" /* MetricChartsComponent */]],
        providers: [__WEBPACK_IMPORTED_MODULE_6__metric_service__["a" /* MetricService */]]
    })
], MetricModule);

var MetricModule_1;
//# sourceMappingURL=metric.module.js.map

/***/ }),

/***/ "../../../../../src/app/metric/metric.service.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__selected_project_service__ = __webpack_require__("../../../../../src/app/selected-project.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__helper_httpService__ = __webpack_require__("../../../../../src/app/helper/httpService.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__angular_http__ = __webpack_require__("../../../http/@angular/http.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4_rxjs_Rx__ = __webpack_require__("../../../../rxjs/Rx.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4_rxjs_Rx___default = __webpack_require__.n(__WEBPACK_IMPORTED_MODULE_4_rxjs_Rx__);
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5__helper_metric__ = __webpack_require__("../../../../../src/app/metric/helper/metric.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_6__helper_metric_data_item__ = __webpack_require__("../../../../../src/app/metric/helper/metric-data-item.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return MetricService; });
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
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};







var MetricService = (function (_super) {
    __extends(MetricService, _super);
    function MetricService(selectedProjectService, http) {
        var _this = _super.call(this) || this;
        _this.selectedProjectService = selectedProjectService;
        _this.http = http;
        _this._metrics = new __WEBPACK_IMPORTED_MODULE_4_rxjs_Rx__["BehaviorSubject"]([]);
        _this.metrics$ = _this._metrics.asObservable();
        selectedProjectService.selectedProject$.subscribe(function (newProject) {
            if (newProject.name !== '') {
                _this.projectPath = newProject.path;
                _this.readData().subscribe(function (data) { return _this.convert(data); });
            }
        });
        return _this;
    }
    MetricService.prototype.getMetrics = function () {
        return this._metrics;
    };
    MetricService.prototype.readData = function () {
        var tablePath = this.projectPath + 'metrics/metrics.json';
        return this.http.get(tablePath)
            .map(__WEBPACK_IMPORTED_MODULE_1__selected_project_service__["a" /* SelectedProjectService */].extractData)
            .catch(__WEBPACK_IMPORTED_MODULE_1__selected_project_service__["a" /* SelectedProjectService */].handleError);
    };
    MetricService.prototype.convert = function (data) {
        var metrics = [];
        for (var _i = 0, data_1 = data; _i < data_1.length; _i++) {
            var metric = data_1[_i];
            var metricName = metric['name'];
            var metricData = [];
            for (var _a = 0, _b = metric['data']; _a < _b.length; _a++) {
                var dataItem = _b[_a];
                var dataItemName = dataItem['artifact'];
                var dataItemType = dataItem['type'];
                var dataItemValue = dataItem['value'];
                metricData.push(new __WEBPACK_IMPORTED_MODULE_6__helper_metric_data_item__["a" /* MetricDataItem */](dataItemName, dataItemType, dataItemValue));
            }
            metrics.push(new __WEBPACK_IMPORTED_MODULE_5__helper_metric__["a" /* Metric */](metricName, metricData));
        }
        this._metrics.next(metrics);
    };
    return MetricService;
}(__WEBPACK_IMPORTED_MODULE_2__helper_httpService__["a" /* HttpService */]));
MetricService = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Injectable"])(),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__selected_project_service__["a" /* SelectedProjectService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__selected_project_service__["a" /* SelectedProjectService */]) === "function" && _a || Object, typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_3__angular_http__["d" /* Http */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_3__angular_http__["d" /* Http */]) === "function" && _b || Object])
], MetricService);

var _a, _b;
//# sourceMappingURL=metric.service.js.map

/***/ }),

/***/ "../../../../../src/app/metric/metric/metric.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, "", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/metric/metric/metric.component.html":
/***/ (function(module, exports) {

module.exports = "<div class=\"container\">\n  <div class=\"row\">\n    <app-metric-table\n      [selectedMetric]=\"selectedMetric\"\n      [coloring]=\"coloring\"\n      (onSubTableChange)=\"subTableChanged($event)\"\n    ></app-metric-table>\n  </div>\n\n  <br/>\n\n  <div class=\"row\">\n\n    <div class=\"col-md-2\">\n\n      <div class=\"d-inline-block\" ngbDropdown #myDrop=\"ngbDropdown\">\n        <a class=\"btn btn-outline-secondary\" id=\"dropdownMenuProjects\" ngbDropdownToggle>{{selectedMetric}}</a>\n        <div class=\"dropdown-menu\" aria-labelledby=\"dropdownMenuProjects\">\n          <a *ngFor='let metric of metricsInDropdown' (click)=\"selectMetricGraphical(metric)\" class=\"dropdown-item\">{{metric}}</a>\n        </div>\n\n      </div>\n\n      <label>\n        Show only Highest Results\n        <input id=\"highestResults\" type=\"checkbox\" [(ngModel)]=\"showOnlyHighestResults\">\n      </label>\n\n      <label>\n        Number of results to show\n        <input id=\"numberResults\"\n               type=\"number\"\n               min=\"1\"\n               max=\"10000\"\n               [ngModel]=\"resultCount\"\n               (ngModelChange)=\"resultCountChanged($event)\"\n               [disabled]=\"!showOnlyHighestResults\">\n      </label>\n\n\n    </div>\n\n    <div class=\"col-md-9\">\n      <app-metric-charts\n        [metricToShow]=\"selectedMetric\"\n        [onlyHighest]=\"showOnlyHighestResults\"\n        [highestCount]=\"resultCount\"\n        [properties]=\"properties\"\n      ></app-metric-charts>\n    </div>\n\n  </div>\n</div>\n"

/***/ }),

/***/ "../../../../../src/app/metric/metric/metric.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__selected_project_service__ = __webpack_require__("../../../../../src/app/selected-project.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__selected_od_service__ = __webpack_require__("../../../../../src/app/selected-od.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return MetricComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};



var MetricComponent = (function () {
    function MetricComponent(selectedProjectService, selectedOdService) {
        var _this = this;
        this.selectedProjectService = selectedProjectService;
        this.selectedOdService = selectedOdService;
        this.showOnlyHighestResults = true;
        this.resultCount = 5;
        this.selectedMetric = 'Metric';
        this.coloring = { active: false, min: 0, max: 0 };
        // TODO Async
        selectedProjectService.properties$.subscribe(function (properties) {
            _this.initProperties(properties);
        });
    }
    MetricComponent.prototype.ngOnInit = function () {
        this.selectedOdService.enableOdDropdown(false);
    };
    MetricComponent.prototype.ngAfterViewInit = function () {
        var _this = this;
        var project = this.selectedProjectService.getSelectedProject().getValue();
        if (project.name !== '') {
            this.selectedProjectService.getProperties().subscribe(function (properties) {
                _this.initProperties(properties);
            });
        }
    };
    MetricComponent.prototype.ngOnDestroy = function () {
        this.selectedOdService.enableOdDropdown(true);
    };
    MetricComponent.prototype.initProperties = function (properties) {
        var _this = this;
        // if ( properties['metrics']) {
        this.properties = properties.metrics.graphical;
        this.metrics = [];
        for (var _i = 0, _a = this.properties; _i < _a.length; _i++) {
            var prop = _a[_i];
            this.metrics.push(prop.name);
        }
        setTimeout(function () {
            var usedOd = _this.selectedProjectService.getOdFromPath(properties.metrics.usedOd);
            _this.selectedOdService.setSelectedOd(usedOd);
        }, 0);
        // }
    };
    MetricComponent.prototype.selectMetricGraphical = function (metric) {
        this.selectedMetric = metric;
        if (this.properties) {
            var min = void 0;
            var max = void 0;
            var active = void 0;
            var prop = this.properties.find(function (m) {
                return m.name === metric;
            });
            if (prop != null && prop.coloring) {
                min = prop.coloringMin;
                max = prop.coloringMax;
                active = true;
            }
            else {
                active = false;
            }
            this.coloring = { active: active, min: min, max: max };
        }
    };
    MetricComponent.prototype.resultCountChanged = function (event) {
        // Avoid double Events
        if (event !== this.resultCount) {
            this.resultCount = event;
        }
    };
    MetricComponent.prototype.subTableChanged = function (event) {
        var _this = this;
        setTimeout(function () {
            _this.metricsInDropdown = _this.metrics.filter(function (e) {
                return event.metrics.indexOf(e) > -1;
            });
            if (_this.metricsInDropdown.indexOf(_this.selectedMetric) === -1) {
                _this.selectedMetric = 'Metric';
            }
        }, 0);
    };
    return MetricComponent;
}());
MetricComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-metric',
        template: __webpack_require__("../../../../../src/app/metric/metric/metric.component.html"),
        styles: [__webpack_require__("../../../../../src/app/metric/metric/metric.component.css")]
    }),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__selected_project_service__["a" /* SelectedProjectService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__selected_project_service__["a" /* SelectedProjectService */]) === "function" && _a || Object, typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_2__selected_od_service__["a" /* SelectedOdService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_2__selected_od_service__["a" /* SelectedOdService */]) === "function" && _b || Object])
], MetricComponent);

var _a, _b;
//# sourceMappingURL=metric.component.js.map

/***/ }),

/***/ "../../../../../src/app/navigation/dropdown/dropdown.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, ".navbar {\n  padding-top: 0;\n  padding-bottom: 5px;\n}\n\n.btn {\n  font-size: 12px;\n}\n\n.navbar-text {\n  margin-top: 0;\n  margin-bottom: 0;\n  margin-left: 5px;\n  margin-right: 5px;\n}\n", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/navigation/dropdown/dropdown.component.html":
/***/ (function(module, exports) {

module.exports = "<nav class=\"navbar navbar-toggleable-md navbar-light bg-faded\">\n\n  <div class=\"d-inline-block\" ngbDropdown #myDrop=\"ngbDropdown\">\n    <a class=\"btn btn-outline-secondary\" id=\"dropdownMenuProjects\" ngbDropdownToggle>{{selectedProject.name}}</a>\n    <div class=\"dropdown-menu\" aria-labelledby=\"dropdownMenuProjects\">\n      <a *ngFor='let project of projects' (click)=\"selectProject(project)\" class=\"dropdown-item\">{{project.name}}</a>\n    </div>\n\n  </div>\n\n  <li class=\"navbar-text\">\n    <i class=\"fa fa-caret-right\" aria-hidden=\"true\"></i>\n  </li>\n\n  <div class=\"d-inline-block\" ngbDropdown #myDrop=\"ngbDropdown\" >\n    <a class=\"btn btn-outline-secondary\"\n       [ngClass]=\"{disabled : odDropdownDisabled}\"\n       id=\"dropdownMenuOds\"\n       ngbDropdownToggle>{{selectedOd.name}}</a>\n    <div class=\"dropdown-menu\" aria-labelledby=\"dropdownMenuOds\">\n      <a *ngFor='let od of odDropdown' (click)=\"selectOd(od)\" class=\"dropdown-item\">{{od.name}}</a>\n    </div>\n\n  </div>\n\n</nav>\n"

/***/ }),

/***/ "../../../../../src/app/navigation/dropdown/dropdown.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__all_projects_service__ = __webpack_require__("../../../../../src/app/all-projects.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__selected_project_service__ = __webpack_require__("../../../../../src/app/selected-project.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__helper_project__ = __webpack_require__("../../../../../src/app/helper/project.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__helper_objectdiagram__ = __webpack_require__("../../../../../src/app/helper/objectdiagram.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5__selected_od_service__ = __webpack_require__("../../../../../src/app/selected-od.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return DropdownComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};






var DropdownComponent = (function () {
    function DropdownComponent(allProjectsService, selectedProjectService, selectedOdService) {
        var _this = this;
        this.allProjectsService = allProjectsService;
        this.selectedProjectService = selectedProjectService;
        this.selectedOdService = selectedOdService;
        this.selectedProject = new __WEBPACK_IMPORTED_MODULE_3__helper_project__["a" /* Project */]('Select Project', '');
        this.selectedOd = new __WEBPACK_IMPORTED_MODULE_4__helper_objectdiagram__["a" /* Objectdiagram */]('Select OD', '', '');
        selectedProjectService.selectedProject$.subscribe(function (selectedProject) {
            _this.selectedProjectChanged(selectedProject);
        });
        selectedProjectService.odList$.subscribe(function () {
            _this.setDisplayableOds();
        });
        selectedOdService.selectedOd$.subscribe(function (selectedOd) {
            _this.selectedOdChanged(selectedOd);
        });
        selectedOdService.odDropdownDisabled$.subscribe(function (isDisabled) { return _this.odDropdownDisabled = isDisabled; });
    }
    DropdownComponent.prototype.ngOnInit = function () {
        var _this = this;
        this.allProjectsService.getProjects()
            .subscribe(function (allProjects) { return _this.projects = allProjects; }, function (error) { return console.log(error); });
    };
    DropdownComponent.prototype.selectOd = function (od) {
        this.selectedOdService.setSelectedOd(od);
    };
    DropdownComponent.prototype.selectProject = function (project) {
        this.selectedProjectService.setSelectedProject(project);
    };
    DropdownComponent.prototype.selectedProjectChanged = function (newProject) {
        this.selectedProject = newProject;
    };
    DropdownComponent.prototype.selectedOdChanged = function (newOd) {
        this.selectedOd = newOd;
    };
    DropdownComponent.prototype.setDisplayableOds = function () {
        var _this = this;
        this.selectedProjectService.getDisplayableOdList().then(function (list) {
            _this.odDropdown = list;
        });
    };
    return DropdownComponent;
}());
DropdownComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-dropdown',
        template: __webpack_require__("../../../../../src/app/navigation/dropdown/dropdown.component.html"),
        styles: [__webpack_require__("../../../../../src/app/navigation/dropdown/dropdown.component.css")]
    }),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__all_projects_service__["a" /* AllProjectsService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__all_projects_service__["a" /* AllProjectsService */]) === "function" && _a || Object, typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_2__selected_project_service__["a" /* SelectedProjectService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_2__selected_project_service__["a" /* SelectedProjectService */]) === "function" && _b || Object, typeof (_c = typeof __WEBPACK_IMPORTED_MODULE_5__selected_od_service__["a" /* SelectedOdService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_5__selected_od_service__["a" /* SelectedOdService */]) === "function" && _c || Object])
], DropdownComponent);

var _a, _b, _c;
//# sourceMappingURL=dropdown.component.js.map

/***/ }),

/***/ "../../../../../src/app/navigation/navigation.module.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__angular_common__ = __webpack_require__("../../../common/@angular/common.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__angular_router__ = __webpack_require__("../../../router/@angular/router.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__navigation_navigation_component__ = __webpack_require__("../../../../../src/app/navigation/navigation/navigation.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__selected_project_service__ = __webpack_require__("../../../../../src/app/selected-project.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5__project_date_project_date_component__ = __webpack_require__("../../../../../src/app/navigation/project-date/project-date.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_6__dropdown_dropdown_component__ = __webpack_require__("../../../../../src/app/navigation/dropdown/dropdown.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_7__ng_bootstrap_ng_bootstrap__ = __webpack_require__("../../../../@ng-bootstrap/ng-bootstrap/index.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_8__all_projects_service__ = __webpack_require__("../../../../../src/app/all-projects.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_9__selected_od_service__ = __webpack_require__("../../../../../src/app/selected-od.service.ts");
/* unused harmony export routes */
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return NavigationModule; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};










var routes = [
    {
        path: '',
        redirectTo: '/textual',
        pathMatch: 'full'
    },
    {
        path: 'textual',
        loadChildren: 'app/textual/textual.module#TextualModule'
    },
    {
        path: 'metrics',
        loadChildren: 'app/metric/metric.module#MetricModule'
    },
    {
        path: 'graph',
        loadChildren: 'app/graph/graph.module#GraphModule'
    },
    {
        path: 'cypher',
        loadChildren: 'app/cypher/cypher.module#CypherModule'
    },
    {
        path: 'modules',
        loadChildren: 'app/modules/modules.module#ModulesModule'
    }
];
var NavigationModule = NavigationModule_1 = (function () {
    function NavigationModule() {
    }
    NavigationModule.forRoot = function () {
        return {
            ngModule: NavigationModule_1,
            providers: [
                __WEBPACK_IMPORTED_MODULE_8__all_projects_service__["a" /* AllProjectsService */],
                __WEBPACK_IMPORTED_MODULE_4__selected_project_service__["a" /* SelectedProjectService */],
                __WEBPACK_IMPORTED_MODULE_9__selected_od_service__["a" /* SelectedOdService */]
            ]
        };
    };
    return NavigationModule;
}());
NavigationModule = NavigationModule_1 = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["NgModule"])({
        imports: [
            __WEBPACK_IMPORTED_MODULE_1__angular_common__["CommonModule"],
            __WEBPACK_IMPORTED_MODULE_7__ng_bootstrap_ng_bootstrap__["a" /* NgbModule */],
            __WEBPACK_IMPORTED_MODULE_2__angular_router__["a" /* RouterModule */].forRoot(routes)
        ],
        exports: [__WEBPACK_IMPORTED_MODULE_2__angular_router__["a" /* RouterModule */], __WEBPACK_IMPORTED_MODULE_3__navigation_navigation_component__["a" /* NavigationComponent */]],
        declarations: [
            __WEBPACK_IMPORTED_MODULE_3__navigation_navigation_component__["a" /* NavigationComponent */],
            __WEBPACK_IMPORTED_MODULE_5__project_date_project_date_component__["a" /* ProjectDateComponent */],
            __WEBPACK_IMPORTED_MODULE_6__dropdown_dropdown_component__["a" /* DropdownComponent */]
        ],
    })
], NavigationModule);

var NavigationModule_1;
//# sourceMappingURL=navigation.module.js.map

/***/ }),

/***/ "../../../../../src/app/navigation/navigation/navigation.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, ".navbar {\n  padding-bottom: 0;\n}\n", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/navigation/navigation/navigation.component.html":
/***/ (function(module, exports) {

module.exports = "<nav class=\"navbar navbar-toggleable-md navbar-light bg-faded\">\n\n  <a class=\"navbar-brand\" routerLink=\"/textual\" (click)=\"resetPage()\">Artifacts</a>\n\n  <div class=\"navbar-collapse\" id=\"navbarMain\">\n    <ul class=\"navbar-nav mr-auto\">\n      <li class=\"nav-item\">\n        <a class=\"nav-link\" routerLink=\"/textual\">Textual</a>\n      </li>\n      <li class=\"nav-item\">\n        <a class=\"nav-link\" [class.disabled]=\"!graphActive\" [routerLink]=\"graphActive ? '/graph' : null\">Graph</a>\n      </li>\n      <li class=\"nav-item\">\n        <a class=\"nav-link\" [class.disabled]=\"!metricActive\" [routerLink]=\"graphActive ? '/metrics' : null\">Metrics</a>\n      </li>\n      <li class=\"nav-item\">\n        <a class=\"nav-link\" routerLink=\"/cypher\">Cypher</a>\n      </li>\n      <li class=\"nav-item\">\n        <a class=\"nav-link\" routerLink=\"/modules\">Modules</a>\n      </li>\n\n\n      <div class=\"alert alert-warning navbar-text\" *ngIf=\"showWarning\">\n        Project Propertyfile not enough specified\n      </div>\n\n    </ul>\n\n    <span class=\"navbar-text\">\n      <app-project-date></app-project-date>\n    </span>\n\n  </div>\n\n</nav>\n\n\n<app-dropdown></app-dropdown>\n"

/***/ }),

/***/ "../../../../../src/app/navigation/navigation/navigation.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__selected_project_service__ = __webpack_require__("../../../../../src/app/selected-project.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return NavigationComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};


var NavigationComponent = (function () {
    function NavigationComponent(selectedProjectService) {
        var _this = this;
        this.selectedProjectService = selectedProjectService;
        selectedProjectService.properties$.subscribe(function (prop) {
            _this.graphActive = prop.graph.tabs.length > 0;
            _this.metricActive = prop.metrics.usedOd !== '';
            _this.showWarning = !_this.graphActive || !_this.metricActive;
        });
    }
    NavigationComponent.prototype.ngOnInit = function () {
    };
    NavigationComponent.prototype.resetPage = function () {
        this.selectedProjectService.setStateToDefault();
    };
    return NavigationComponent;
}());
NavigationComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-navigation',
        template: __webpack_require__("../../../../../src/app/navigation/navigation/navigation.component.html"),
        styles: [__webpack_require__("../../../../../src/app/navigation/navigation/navigation.component.css")]
    }),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__selected_project_service__["a" /* SelectedProjectService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__selected_project_service__["a" /* SelectedProjectService */]) === "function" && _a || Object])
], NavigationComponent);

var _a;
//# sourceMappingURL=navigation.component.js.map

/***/ }),

/***/ "../../../../../src/app/navigation/project-date/project-date.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, "", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/navigation/project-date/project-date.component.html":
/***/ (function(module, exports) {

module.exports = "{{date | date:'medium' }}\n"

/***/ }),

/***/ "../../../../../src/app/navigation/project-date/project-date.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__selected_project_service__ = __webpack_require__("../../../../../src/app/selected-project.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return ProjectDateComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};


var ProjectDateComponent = (function () {
    function ProjectDateComponent(selectedProjectService) {
        var _this = this;
        this.selectedProjectService = selectedProjectService;
        selectedProjectService.projectDate$.subscribe(function (date) {
            _this.date = date;
        });
    }
    ProjectDateComponent.prototype.ngOnInit = function () {
    };
    return ProjectDateComponent;
}());
ProjectDateComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-project-date',
        template: __webpack_require__("../../../../../src/app/navigation/project-date/project-date.component.html"),
        styles: [__webpack_require__("../../../../../src/app/navigation/project-date/project-date.component.css")],
    }),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__selected_project_service__["a" /* SelectedProjectService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__selected_project_service__["a" /* SelectedProjectService */]) === "function" && _a || Object])
], ProjectDateComponent);

var _a;
//# sourceMappingURL=project-date.component.js.map

/***/ }),

/***/ "../../../../../src/app/selected-od.service.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__helper_httpService__ = __webpack_require__("../../../../../src/app/helper/httpService.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2_rxjs_Rx__ = __webpack_require__("../../../../rxjs/Rx.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2_rxjs_Rx___default = __webpack_require__.n(__WEBPACK_IMPORTED_MODULE_2_rxjs_Rx__);
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__angular_http__ = __webpack_require__("../../../http/@angular/http.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__selected_project_service__ = __webpack_require__("../../../../../src/app/selected-project.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return SelectedOdService; });
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
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};





var SelectedOdService = SelectedOdService_1 = (function (_super) {
    __extends(SelectedOdService, _super);
    function SelectedOdService(http, selectedProjectService) {
        var _this = _super.call(this) || this;
        _this.http = http;
        _this.selectedProjectService = selectedProjectService;
        _this._selectedOd = new __WEBPACK_IMPORTED_MODULE_2_rxjs_Rx__["Subject"]();
        _this._odDropdownDisabled = new __WEBPACK_IMPORTED_MODULE_2_rxjs_Rx__["BehaviorSubject"](false);
        _this.selectedOd$ = _this._selectedOd.asObservable();
        _this.odDropdownDisabled$ = _this._odDropdownDisabled.asObservable();
        _this.jsonData = JSON.parse('{"elements" : [], "links": []}');
        selectedProjectService.folderStructure$.subscribe(function () {
            // Set default OD
            selectedProjectService.getProperties().subscribe(function (properties) {
                var defaultOdPath = selectedProjectService.getSelectedProject().getValue().path + properties.main.defaultFile;
                var defaultOd = selectedProjectService.getOdFromPath(defaultOdPath);
                if (defaultOd != null) {
                    _this.setSelectedOd(defaultOd);
                }
            });
        }, function (error) { return console.log(error); });
        return _this;
    }
    SelectedOdService.prototype.enableOdDropdown = function (isEnabled) {
        this._odDropdownDisabled.next(!isEnabled);
    };
    SelectedOdService.prototype.setSelectedOd = function (od) {
        var _this = this;
        this.selectedProjectService.getDisplayableOdList().then(function (odList) {
            if (odList.some(function (element) {
                return element.name === od.name;
            })) {
                _this._selectedOdData = od;
                _this.loadJSON().subscribe(function (data) {
                    _this.jsonData = data;
                    _this._selectedOd.next(od);
                });
            }
        });
    };
    SelectedOdService.prototype.getSelectedOd = function () {
        return this._selectedOdData;
    };
    SelectedOdService.prototype.loadJSON = function () {
        var od = this.getSelectedOd();
        if (od == null) {
            return __WEBPACK_IMPORTED_MODULE_2_rxjs_Rx__["Observable"].empty();
        }
        var path = od.jsonPath;
        return this.http.get(path)
            .map(SelectedOdService_1.extractData)
            .catch(SelectedOdService_1.handleError);
    };
    SelectedOdService.prototype.getJsonData = function () {
        return __WEBPACK_IMPORTED_MODULE_2_rxjs_Rx__["Observable"].of(this.jsonData);
    };
    return SelectedOdService;
}(__WEBPACK_IMPORTED_MODULE_1__helper_httpService__["a" /* HttpService */]));
SelectedOdService = SelectedOdService_1 = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Injectable"])(),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_3__angular_http__["d" /* Http */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_3__angular_http__["d" /* Http */]) === "function" && _a || Object, typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_4__selected_project_service__["a" /* SelectedProjectService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_4__selected_project_service__["a" /* SelectedProjectService */]) === "function" && _b || Object])
], SelectedOdService);

var SelectedOdService_1, _a, _b;
//# sourceMappingURL=selected-od.service.js.map

/***/ }),

/***/ "../../../../../src/app/selected-project.service.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__helper_project__ = __webpack_require__("../../../../../src/app/helper/project.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__helper_objectdiagram__ = __webpack_require__("../../../../../src/app/helper/objectdiagram.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__all_projects_service__ = __webpack_require__("../../../../../src/app/all-projects.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__angular_http__ = __webpack_require__("../../../http/@angular/http.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5_rxjs_Rx__ = __webpack_require__("../../../../rxjs/Rx.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5_rxjs_Rx___default = __webpack_require__.n(__WEBPACK_IMPORTED_MODULE_5_rxjs_Rx__);
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_6__helper_httpService__ = __webpack_require__("../../../../../src/app/helper/httpService.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_7__helper_properties__ = __webpack_require__("../../../../../src/app/helper/properties.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_8__helper_mainProperties__ = __webpack_require__("../../../../../src/app/helper/mainProperties.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return SelectedProjectService; });
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
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};









var SelectedProjectService = SelectedProjectService_1 = (function (_super) {
    __extends(SelectedProjectService, _super);
    function SelectedProjectService(allProjectsService, http) {
        var _this = _super.call(this) || this;
        _this.allProjectsService = allProjectsService;
        _this.http = http;
        _this._selectedProject = new __WEBPACK_IMPORTED_MODULE_5_rxjs_Rx__["BehaviorSubject"](new __WEBPACK_IMPORTED_MODULE_1__helper_project__["a" /* Project */]('', ''));
        _this._odList = new __WEBPACK_IMPORTED_MODULE_5_rxjs_Rx__["BehaviorSubject"]([]);
        _this._folderStructure = new __WEBPACK_IMPORTED_MODULE_5_rxjs_Rx__["BehaviorSubject"](JSON.parse('[]'));
        _this._projectDate = new __WEBPACK_IMPORTED_MODULE_5_rxjs_Rx__["BehaviorSubject"](new Date());
        _this._properties = new __WEBPACK_IMPORTED_MODULE_5_rxjs_Rx__["BehaviorSubject"](new __WEBPACK_IMPORTED_MODULE_7__helper_properties__["a" /* Properties */]());
        _this.selectedProject$ = _this._selectedProject.asObservable();
        _this.odList$ = _this._odList.asObservable();
        _this.folderStructure$ = _this._folderStructure.asObservable();
        _this.projectDate$ = _this._projectDate.asObservable();
        _this.properties$ = _this._properties.asObservable();
        _this.propertiesUpToDate = true;
        _this.setStateToDefault();
        return _this;
    }
    SelectedProjectService.prototype.onGlobalPropertiesRead = function (globalProperties) {
        this.globalProperties = globalProperties;
        var defaultProjectName = this.globalProperties.defaultProject;
        var defaultProjectPath = 'assets/' + defaultProjectName + '/';
        var defaultProject = new __WEBPACK_IMPORTED_MODULE_1__helper_project__["a" /* Project */](defaultProjectName, defaultProjectPath);
        this.setSelectedProject(defaultProject);
    };
    SelectedProjectService.prototype.setStateToDefault = function () {
        var _this = this;
        this.allProjectsService.getGlobalProperties()
            .subscribe(function (properties) { return _this.onGlobalPropertiesRead(properties); }, function (error) { return console.log(error); });
    };
    SelectedProjectService.prototype.parseOdList = function (json) {
        var result = [];
        for (var _i = 0, json_1 = json; _i < json_1.length; _i++) {
            var jsonObject = json_1[_i];
            var pathToCut = this.getSelectedProject().getValue().path + 'textFiles/';
            var name = jsonObject['name'].substr(pathToCut.length);
            var path = jsonObject['name'];
            var jsonPath = jsonObject['jsonPath'];
            var od = new __WEBPACK_IMPORTED_MODULE_2__helper_objectdiagram__["a" /* Objectdiagram */](name, path, jsonPath);
            result.push(od);
        }
        result.sort(function (a, b) {
            return a.name.localeCompare(b.name);
        });
        return result;
    };
    SelectedProjectService.prototype.getDisplayableOdList = function () {
        var _this = this;
        var result = this.getOdList().getValue();
        return new Promise(function (resolve) {
            _this.getProperties().subscribe(function (prop) {
                if (prop.main.showAllDropdownElements) {
                    resolve(result);
                }
                else {
                    result = result.filter(function (od) {
                        return prop.main.dropdownElements.indexOf(od.name) > -1;
                    });
                    resolve(result);
                }
            });
        });
    };
    SelectedProjectService.prototype.getSelectedProject = function () {
        return this._selectedProject;
    };
    SelectedProjectService.prototype.getOdList = function () {
        return this._odList;
    };
    SelectedProjectService.prototype.setSelectedProject = function (project) {
        var _this = this;
        this.propertiesUpToDate = false;
        this._selectedProject.next(project);
        // Load Date
        this.getDate().then(function (date) { return _this._projectDate.next(date); }, function (error) { return console.log(error); });
        // Load OD List
        var dataPath = project.path + 'textFiles';
        this.http.post('getAllOd.php', dataPath)
            .map(SelectedProjectService_1.extractData)
            .catch(SelectedProjectService_1.handleError)
            .subscribe(function (allOds) {
            _this.setOds(_this.parseOdList(allOds));
            // Load Properties
            _this.getProperties().subscribe(function (properties) {
                _this._properties.next(properties);
                _this.propertiesUpToDate = true;
                // Load Folder Structure
                _this.makeFolderStructure().then(function (folderStructure) {
                    _this.setFolderStructure(folderStructure);
                });
            }, function (error) { return console.log(error); });
        }, function (error) { return console.log(error); });
    };
    SelectedProjectService.prototype.loadProperties = function () {
        var propertiesPath = this.getSelectedProject().getValue().path + 'properties.json';
        return this.http.get(propertiesPath)
            .map(function (res) {
            var json = SelectedProjectService_1.extractData(res);
            return __WEBPACK_IMPORTED_MODULE_7__helper_properties__["a" /* Properties */].convertJSONToObject(json);
        })
            .catch(SelectedProjectService_1.handleErrorWith404Acceptance);
    };
    SelectedProjectService.prototype.getProperties = function () {
        var _this = this;
        if (this.propertiesUpToDate) {
            var properties = this._properties.getValue();
            return __WEBPACK_IMPORTED_MODULE_5_rxjs_Rx__["Observable"].of(properties);
        }
        else {
            return new __WEBPACK_IMPORTED_MODULE_5_rxjs_Rx__["Observable"](function (observer) {
                _this.loadProperties().subscribe(function (prop) {
                    _this.resolveProperties(prop).then(function (resProp) {
                        observer.next(resProp);
                    });
                });
            });
        }
    };
    SelectedProjectService.prototype.resolveProperties = function (prop) {
        var _this = this;
        return new Promise(function (resolve) {
            if (prop == null) {
                _this.getFirstOdPath().then(function (path) {
                    var mainProp = new __WEBPACK_IMPORTED_MODULE_8__helper_mainProperties__["a" /* MainProperties */](path);
                    resolve(new __WEBPACK_IMPORTED_MODULE_7__helper_properties__["a" /* Properties */](mainProp));
                });
            }
            else {
                if (!(prop.main.defaultFile && prop.main.defaultFile !== '')) {
                    _this.getFirstOdPath().then(function (path) {
                        var mainProp = new __WEBPACK_IMPORTED_MODULE_8__helper_mainProperties__["a" /* MainProperties */](path, prop.main.showAllDropdownElements, prop.main.dropdownElements);
                        resolve(new __WEBPACK_IMPORTED_MODULE_7__helper_properties__["a" /* Properties */](mainProp, prop.textual, prop.graph, prop.metrics));
                    });
                }
                else {
                    resolve(prop);
                }
            }
        });
    };
    SelectedProjectService.prototype.getFirstOdPath = function () {
        var _this = this;
        return new Promise(function (resolve) {
            _this.getOdList().subscribe(function (list) {
                if (list.length > 0) {
                    var path = list[0].path.substring(_this.getSelectedProject().getValue().path.length);
                    resolve(path);
                }
                else {
                    resolve('');
                }
            });
        });
    };
    SelectedProjectService.prototype.setOds = function (ods) {
        this._odList.next(ods);
    };
    SelectedProjectService.prototype.setFolderStructure = function (folderStructure) {
        this._folderStructure.next(folderStructure);
    };
    SelectedProjectService.prototype.getDate = function () {
        var _this = this;
        return new Promise(function (resolve, reject) {
            var projectPath = _this.getSelectedProject().getValue().path;
            if (projectPath === '') {
                reject('No Project');
            }
            var path = projectPath + 'textFiles/';
            _this.http.post('getFileDate.php', path)
                .map(SelectedProjectService_1.extractData)
                .catch(SelectedProjectService_1.handleError)
                .subscribe(function (date) {
                var result = new Date(date * 1000);
                resolve(result);
            });
        });
    };
    SelectedProjectService.prototype.makeFolderStructure = function () {
        var _this = this;
        return new Promise(function (resolve, reject) {
            var projectRoot = _this.getSelectedProject().getValue().path;
            _this.getProperties().subscribe(function (prop) {
                _this.http.post('convertFolder.php', projectRoot)
                    .map(SelectedProjectService_1.extractData)
                    .catch(SelectedProjectService_1.handleError)
                    .subscribe(function (folderStructure) {
                    resolve(_this.translateFolderStructure(folderStructure, prop.textual));
                }, function (error) {
                    console.log(error);
                    reject(error);
                });
            });
        });
    };
    SelectedProjectService.prototype.translateFolderStructure = function (json, prop) {
        var _this = this;
        var result = json;
        var fileExtensions = prop.extensions;
        if (fileExtensions != null) {
            result = json.filter(function (e) {
                if (e['ext'] === 'folder') {
                    e['children'] = _this.translateFolderStructure(e['children'], prop);
                    return e['children'].length > 0;
                }
                return fileExtensions.indexOf(e['ext']) !== -1;
            });
        }
        return result;
    };
    SelectedProjectService.prototype.getOdFromPath = function (path) {
        var odList = this.getOdList().getValue();
        for (var _i = 0, odList_1 = odList; _i < odList_1.length; _i++) {
            var od = odList_1[_i];
            if (od.path.endsWith(path)) {
                return od;
            }
        }
        return null;
    };
    return SelectedProjectService;
}(__WEBPACK_IMPORTED_MODULE_6__helper_httpService__["a" /* HttpService */]));
SelectedProjectService = SelectedProjectService_1 = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Injectable"])(),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_3__all_projects_service__["a" /* AllProjectsService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_3__all_projects_service__["a" /* AllProjectsService */]) === "function" && _a || Object, typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_4__angular_http__["d" /* Http */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_4__angular_http__["d" /* Http */]) === "function" && _b || Object])
], SelectedProjectService);

var SelectedProjectService_1, _a, _b;
//# sourceMappingURL=selected-project.service.js.map

/***/ }),

/***/ "../../../../../src/app/textual/file-content.service.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__angular_http__ = __webpack_require__("../../../http/@angular/http.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__helper_httpService__ = __webpack_require__("../../../../../src/app/helper/httpService.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return FileContentService; });
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
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};



var FileContentService = FileContentService_1 = (function (_super) {
    __extends(FileContentService, _super);
    function FileContentService(http) {
        var _this = _super.call(this) || this;
        _this.http = http;
        return _this;
    }
    // Overrides
    FileContentService.extractData = function (res) {
        return res.text() || ' ';
    };
    FileContentService.prototype.getContent = function (path) {
        return this.http.get(path)
            .map(FileContentService_1.extractData)
            .catch(FileContentService_1.handleError);
    };
    return FileContentService;
}(__WEBPACK_IMPORTED_MODULE_2__helper_httpService__["a" /* HttpService */]));
FileContentService = FileContentService_1 = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Injectable"])(),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__angular_http__["d" /* Http */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__angular_http__["d" /* Http */]) === "function" && _a || Object])
], FileContentService);

var FileContentService_1, _a;
//# sourceMappingURL=file-content.service.js.map

/***/ }),

/***/ "../../../../../src/app/textual/textual-routing.module.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__angular_router__ = __webpack_require__("../../../router/@angular/router.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__textual_textual_component__ = __webpack_require__("../../../../../src/app/textual/textual/textual.component.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return TextualRoutingModule; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};



var routes = [
    { path: '', component: __WEBPACK_IMPORTED_MODULE_2__textual_textual_component__["a" /* TextualComponent */] }
];
var TextualRoutingModule = (function () {
    function TextualRoutingModule() {
    }
    return TextualRoutingModule;
}());
TextualRoutingModule = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["NgModule"])({
        imports: [__WEBPACK_IMPORTED_MODULE_1__angular_router__["a" /* RouterModule */].forChild(routes)],
        exports: [__WEBPACK_IMPORTED_MODULE_1__angular_router__["a" /* RouterModule */]]
    })
], TextualRoutingModule);

//# sourceMappingURL=textual-routing.module.js.map

/***/ }),

/***/ "../../../../../src/app/textual/textual.module.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
Object.defineProperty(__webpack_exports__, "__esModule", { value: true });
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__angular_common__ = __webpack_require__("../../../common/@angular/common.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2_angular_tree_component__ = __webpack_require__("../../../../angular-tree-component/dist/angular-tree-component.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__textual_textual_component__ = __webpack_require__("../../../../../src/app/textual/textual/textual.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_4__textual_routing_module__ = __webpack_require__("../../../../../src/app/textual/textual-routing.module.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_5__tree_tree_component__ = __webpack_require__("../../../../../src/app/textual/tree/tree.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_6__view_view_component__ = __webpack_require__("../../../../../src/app/textual/view/view.component.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_7__selected_project_service__ = __webpack_require__("../../../../../src/app/selected-project.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_8__selected_od_service__ = __webpack_require__("../../../../../src/app/selected-od.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_9__ng_bootstrap_ng_bootstrap__ = __webpack_require__("../../../../@ng-bootstrap/ng-bootstrap/index.js");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "TextualModule", function() { return TextualModule; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};










var TextualModule = TextualModule_1 = (function () {
    function TextualModule() {
    }
    TextualModule.forRoot = function () {
        return {
            ngModule: TextualModule_1,
            providers: [__WEBPACK_IMPORTED_MODULE_7__selected_project_service__["a" /* SelectedProjectService */], __WEBPACK_IMPORTED_MODULE_8__selected_od_service__["a" /* SelectedOdService */]]
        };
    };
    return TextualModule;
}());
TextualModule = TextualModule_1 = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["NgModule"])({
        imports: [
            __WEBPACK_IMPORTED_MODULE_1__angular_common__["CommonModule"],
            __WEBPACK_IMPORTED_MODULE_4__textual_routing_module__["a" /* TextualRoutingModule */],
            __WEBPACK_IMPORTED_MODULE_2_angular_tree_component__["a" /* TreeModule */],
            __WEBPACK_IMPORTED_MODULE_9__ng_bootstrap_ng_bootstrap__["a" /* NgbModule */]
        ],
        declarations: [__WEBPACK_IMPORTED_MODULE_3__textual_textual_component__["a" /* TextualComponent */], __WEBPACK_IMPORTED_MODULE_5__tree_tree_component__["a" /* TreeComponent */], __WEBPACK_IMPORTED_MODULE_6__view_view_component__["a" /* ViewComponent */]]
    })
], TextualModule);

var TextualModule_1;
//# sourceMappingURL=textual.module.js.map

/***/ }),

/***/ "../../../../../src/app/textual/textual/textual.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, "", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/textual/textual/textual.component.html":
/***/ (function(module, exports) {

module.exports = "<div class=\"container\">\n  <div class=\"row\">\n    <div class=\"col-md-4\">\n      <h1>Textual</h1>\n      <app-tree #tree></app-tree>\n    </div>\n\n    <div class=\"col-md-8\">\n      <app-view [element]=\"tree.selectedElement\"></app-view>\n    </div>\n  </div>\n</div>\n\n"

/***/ }),

/***/ "../../../../../src/app/textual/textual/textual.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return TextualComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};

var TextualComponent = (function () {
    function TextualComponent() {
    }
    TextualComponent.prototype.ngOnInit = function () {
    };
    return TextualComponent;
}());
TextualComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-textual',
        template: __webpack_require__("../../../../../src/app/textual/textual/textual.component.html"),
        styles: [__webpack_require__("../../../../../src/app/textual/textual/textual.component.css")],
    }),
    __metadata("design:paramtypes", [])
], TextualComponent);

//# sourceMappingURL=textual.component.js.map

/***/ }),

/***/ "../../../../../src/app/textual/tree/tree.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, ".scroll-box {\n  overflow-y: scroll;\n  max-height: 60vh;\n  padding: 1rem\n}\n", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/textual/tree/tree.component.html":
/***/ (function(module, exports) {

module.exports = "<div class=\"card scroll-box\">\n  <tree-root #tree [nodes]=\"nodes\"\n           (onActivate)=\"selectElement($event)\"></tree-root>\n</div>\n"

/***/ }),

/***/ "../../../../../src/app/textual/tree/tree.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__selected_project_service__ = __webpack_require__("../../../../../src/app/selected-project.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__selected_od_service__ = __webpack_require__("../../../../../src/app/selected-od.service.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3_angular_tree_component__ = __webpack_require__("../../../../angular-tree-component/dist/angular-tree-component.js");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return TreeComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};




var TreeComponent = (function () {
    function TreeComponent(selectedProjectService, selectedOdService) {
        var _this = this;
        this.selectedProjectService = selectedProjectService;
        this.selectedOdService = selectedOdService;
        this.selectedElement = '';
        selectedProjectService.selectedProject$.subscribe(function (selectedProject) {
            _this.selectedProjectChanged(selectedProject);
        });
        selectedProjectService.folderStructure$.subscribe(function (folderStructure) {
            _this.nodes = folderStructure;
        });
        selectedOdService.selectedOd$.subscribe(function (selectedOd) {
            if (_this.selectedElement.path !== selectedOd.path) {
                // Timeout to give the TreeView Lib Time to Initialize
                setTimeout(function () { return _this.selectedOdChanged(selectedOd); }, 0);
            }
        });
    }
    TreeComponent.prototype.ngOnInit = function () {
    };
    TreeComponent.prototype.ngAfterViewInit = function () {
        var _this = this;
        var project = this.selectedProjectService.getSelectedProject().getValue();
        if (project.name !== '') {
            this.selectedProjectChanged(project);
            var od_1 = this.selectedOdService.getSelectedOd();
            if (od_1 != null && this.nodes != null) {
                // Timeout to give the TreeView Lib Time to Initialize
                setTimeout(function () { return _this.selectedOdChanged(od_1); }, 0);
            }
        }
    };
    TreeComponent.prototype.selectElement = function ($event) {
        // Is not a folder
        var element = $event.node.data;
        if (!element.children) {
            this.selectedElement = element;
            var od = this.selectedProjectService.getOdFromPath(element.path);
            if (od) {
                this.selectedOdService.setSelectedOd(od);
            }
        }
    };
    TreeComponent.prototype.selectedProjectChanged = function (newProject) {
        this.selectedElement = '';
    };
    TreeComponent.prototype.selectedOdChanged = function (newOd) {
        this.activateNode(newOd, this.nodes);
    };
    TreeComponent.prototype.activateNode = function (newOd, tree) {
        var _loop_1 = function (node) {
            if (node['path'] === newOd.path) {
                var treeNode_1 = this_1.tree.treeModel.getNodeById(node['id']);
                setTimeout(function () {
                    treeNode_1.setActiveAndVisible();
                    return;
                }, 0);
            }
            else if (newOd.path.startsWith(node['path'])) {
                if (node.hasOwnProperty('children')) {
                    var children = node['children'];
                    this_1.activateNode(newOd, children);
                    return { value: void 0 };
                }
            }
        };
        var this_1 = this;
        for (var _i = 0, tree_1 = tree; _i < tree_1.length; _i++) {
            var node = tree_1[_i];
            var state_1 = _loop_1(node);
            if (typeof state_1 === "object")
                return state_1.value;
        }
    };
    return TreeComponent;
}());
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["ViewChild"])(__WEBPACK_IMPORTED_MODULE_3_angular_tree_component__["b" /* TreeComponent */]),
    __metadata("design:type", typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_3_angular_tree_component__["b" /* TreeComponent */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_3_angular_tree_component__["b" /* TreeComponent */]) === "function" && _a || Object)
], TreeComponent.prototype, "tree", void 0);
TreeComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-tree',
        template: __webpack_require__("../../../../../src/app/textual/tree/tree.component.html"),
        styles: [__webpack_require__("../../../../../src/app/textual/tree/tree.component.css")],
        // For Custom CSS in Tree
        encapsulation: __WEBPACK_IMPORTED_MODULE_0__angular_core__["ViewEncapsulation"].None,
    }),
    __metadata("design:paramtypes", [typeof (_b = typeof __WEBPACK_IMPORTED_MODULE_1__selected_project_service__["a" /* SelectedProjectService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__selected_project_service__["a" /* SelectedProjectService */]) === "function" && _b || Object, typeof (_c = typeof __WEBPACK_IMPORTED_MODULE_2__selected_od_service__["a" /* SelectedOdService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_2__selected_od_service__["a" /* SelectedOdService */]) === "function" && _c || Object])
], TreeComponent);

var _a, _b, _c;
//# sourceMappingURL=tree.component.js.map

/***/ }),

/***/ "../../../../../src/app/textual/view/view.component.css":
/***/ (function(module, exports, __webpack_require__) {

exports = module.exports = __webpack_require__("../../../../css-loader/lib/css-base.js")(false);
// imports


// module
exports.push([module.i, ".scroll-box {\n  overflow-y: scroll;\n  max-height: 60vh;\n  padding: 1rem;\n  margin-top: 1em;\n}\n", ""]);

// exports


/*** EXPORTS FROM exports-loader ***/
module.exports = module.exports.toString();

/***/ }),

/***/ "../../../../../src/app/textual/view/view.component.html":
/***/ (function(module, exports) {

module.exports = "<h2 *ngIf=\"element.name\" class=\"bg-info\">{{element.name}}\n  <a class=\"pull-right text-center\" href={{element.path}} download={{element.path}} >\n    <i class=\"fa fa-download\" aria-hidden=\"true\"></i>\n  </a>\n</h2>\n\n<!--\n<pre *ngIf=\"content\" class=\"card card-block scroll-box\">{{ content }}</pre>\n-->\n\n<div *ngIf=\"content\">\n\n  <input  #search *ngIf=\"numberPages>1\"\n         class=\"form-control\"\n         type=\"search\"\n         (keyup.enter)=\"searchWord(search.value); search.value=''\"\n         placeholder=\"Search page of first occurrence ...\">\n\n  <pre class=\"card card-block scroll-box\"> {{pageContent[currentPage-1]}} </pre>\n\n  <ngb-pagination *ngIf=\"numberPages>1\"\n    [collectionSize]=\"numberPages\"\n    [pageSize]=\"1\"\n    [(page)]=\"currentPage\"\n    [maxSize]=\"10\"\n    [rotate]=\"true\"\n    [boundaryLinks]=\"true\"\n  ></ngb-pagination>\n\n</div>\n"

/***/ }),

/***/ "../../../../../src/app/textual/view/view.component.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__file_content_service__ = __webpack_require__("../../../../../src/app/textual/file-content.service.ts");
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return ViewComponent; });
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};


var ViewComponent = (function () {
    function ViewComponent(fileContentService) {
        this.fileContentService = fileContentService;
        this.pageContent = [];
        this.maxLinesPerPage = 5000;
    }
    ViewComponent.prototype.ngOnInit = function () {
    };
    ViewComponent.prototype.ngOnChanges = function (changes) {
        var _this = this;
        if (this.element.path) {
            this.fileContentService.getContent(this.element.path)
                .subscribe(function (content) {
                _this.content = content;
                _this.paginate();
            }, function (error) { return console.log(error); });
        }
    };
    ViewComponent.prototype.paginate = function () {
        var maxLinesPerPage = this.maxLinesPerPage;
        this.currentPage = 1;
        var lines = this.content.split(/\r\n|\r|\n/);
        this.numberPages = Math.ceil(lines.length / maxLinesPerPage);
        for (var i = 0; i < this.numberPages; i++) {
            this.pageContent[i] = '';
            for (var j = 0; j < maxLinesPerPage; j++) {
                var line = lines[i * maxLinesPerPage + j];
                if (line == null) {
                    break;
                }
                this.pageContent[i] += lines[i * maxLinesPerPage + j] + '\n';
            }
        }
    };
    ViewComponent.prototype.searchWord = function (searchItem) {
        console.log('Search triggered: ' + searchItem);
        if (this.content.search(searchItem) > -1) {
            for (var i = 0; i < this.numberPages; i++) {
                var isOnPage = this.pageContent[i].search(searchItem);
                if (isOnPage > -1) {
                    this.currentPage = i + 1;
                    return;
                }
            }
        }
    };
    return ViewComponent;
}());
__decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Input"])(),
    __metadata("design:type", Object)
], ViewComponent.prototype, "element", void 0);
ViewComponent = __decorate([
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["Component"])({
        selector: 'app-view',
        template: __webpack_require__("../../../../../src/app/textual/view/view.component.html"),
        styles: [__webpack_require__("../../../../../src/app/textual/view/view.component.css")],
        providers: [__WEBPACK_IMPORTED_MODULE_1__file_content_service__["a" /* FileContentService */]]
    }),
    __metadata("design:paramtypes", [typeof (_a = typeof __WEBPACK_IMPORTED_MODULE_1__file_content_service__["a" /* FileContentService */] !== "undefined" && __WEBPACK_IMPORTED_MODULE_1__file_content_service__["a" /* FileContentService */]) === "function" && _a || Object])
], ViewComponent);

var _a;
//# sourceMappingURL=view.component.js.map

/***/ }),

/***/ "../../../../../src/environments/environment.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, "a", function() { return environment; });
// The file contents for the current environment will overwrite these during build.
// The build system defaults to the dev environment which uses `environment.ts`, but if you do
// `ng build --env=prod` then `environment.prod.ts` will be used instead.
// The list of which env maps to which file can be found in `.angular-cli.json`.
// The file contents for the current environment will overwrite these during build.
var environment = {
    production: false
};
//# sourceMappingURL=environment.js.map

/***/ }),

/***/ "../../../../../src/main.ts":
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
Object.defineProperty(__webpack_exports__, "__esModule", { value: true });
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_0__angular_core__ = __webpack_require__("../../../core/@angular/core.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_1__angular_platform_browser_dynamic__ = __webpack_require__("../../../platform-browser-dynamic/@angular/platform-browser-dynamic.es5.js");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_2__app_app_module__ = __webpack_require__("../../../../../src/app/app.module.ts");
/* harmony import */ var __WEBPACK_IMPORTED_MODULE_3__environments_environment__ = __webpack_require__("../../../../../src/environments/environment.ts");




if (__WEBPACK_IMPORTED_MODULE_3__environments_environment__["a" /* environment */].production) {
    __webpack_require__.i(__WEBPACK_IMPORTED_MODULE_0__angular_core__["enableProdMode"])();
}
__webpack_require__.i(__WEBPACK_IMPORTED_MODULE_1__angular_platform_browser_dynamic__["a" /* platformBrowserDynamic */])().bootstrapModule(__WEBPACK_IMPORTED_MODULE_2__app_app_module__["a" /* AppModule */]);
//# sourceMappingURL=main.js.map

/***/ }),

/***/ 1:
/***/ (function(module, exports, __webpack_require__) {

module.exports = __webpack_require__("../../../../../src/main.ts");


/***/ })

},[1]);
//# sourceMappingURL=main.bundle.js.map