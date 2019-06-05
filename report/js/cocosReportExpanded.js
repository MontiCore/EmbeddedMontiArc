function createTable_custom(file, infoFile) {
    loadInfo(file, infoFile, createTable);
}

function loadInfo(file, infoFile, callback) {
    var xobj = new XMLHttpRequest();
    xobj.overrideMimeType('application/json');
    xobj.open('GET', infoFile, true);
    var res;
    var ready = 0;
    xobj.onreadystatechange = function () {
        if (xobj.readyState == 4 && xobj.status == '200') {
            res = JSON.parse(xobj.responseText);
            loadJSON(file, res, callback);
        }
    };
    xobj.send(null);
}

function loadJSON(file, info, callback) {
    var xobj = new XMLHttpRequest();
    xobj.overrideMimeType('application/json');
    xobj.open('GET', file, true);
    var res;
    var ready = 0;
    xobj.onreadystatechange = function () {
        if (xobj.readyState == 4 && xobj.status == '200') {
            // Required use of an anonymous callback as .open will NOT return a value but simply returns undefined in asynchronous mode
            res = JSON.parse(xobj.responseText);
            callback(res, info);
        }
    };
    xobj.send(null);
}

function createTable(data, info) {
    table = $(tableReference).DataTable({
        "fnInitComplete": function() { init(); },
        "fixedHeader": true,
        "select": true,
        "data": data,
        "orderCellsTop": true,
        "ordering": true,
        "paging": false,
        "bInfo": false,
        "orderFixed": { "pre": [0, 'asc']},
        "rowGroup": {
            "dataSrc": "Root",
            "startRender": function (rows, group) {
                var count = info[group]['Number'];
                var valid = info[group]['Valid'];
                var invalid = info[group]['Invalid'];
                return '<div class="nowrap" style="width:0px; overflow:visible;">' +
                    '<span class="nowrap">' + group + ' (' + count + ')</span>' +
                    '&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ' +
                    '<span class="nowrap">valid: ' + valid + '</span>' +
                    '&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ' +
                    '<span class="nowrap"> invalid: ' + invalid + '</span></div>';
            }
        },
        "aoColumns": [
            {"data": "Root", "visible": false},
            {
                "className": "expandChildren-control",
                "data": "ChildExpansion",
                "orderable": false,
            },
            {
                "data": "Name",
                "bSortable": true,
            },
            // {
            //     "data": "Visualisation",
            //     "className": "visualisation",
            //     "bSortable": true,
            //     "sType": "vis"
            // },
            // {
            //     "data": "OnlineIDE",
            //     "className": "onlineIDE",
            //     "orderable": false,
            //     "visible": ide
            // },
            {
                "className": 'details-control',
                "orderable": false,
                "data": null,
                "defaultContent": ''
            },
            {"data": "FileType", sort: "string"},
            {"data": "Valid", sort: "string", type: "alt-string"},
            {"data": "Parse", sort: "string", type: "alt-string"},
            {"data": "Resolve", sort: "string", type: "alt-string"},
            {"data": "ComponentCapitalized", sort: "string", type: "alt-string"},
            {"data": "ComponentInstanceNamesUnique", sort: "string", type: "alt-string"},
            {"data": "ComponentWithTypeParametersHasInstance", sort: "string", type: "alt-string"},
            {"data": "ConnectorEndPointCorrectlyQualified", sort: "string", type: "alt-string"},
            {"data": "DefaultParametersHaveCorrectOrder", sort: "string", type: "alt-string"},
            {"data": "InPortUniqueSender", sort: "string", type: "alt-string"},
            {"data": "PackageLowerCase", sort: "string", type: "alt-string"},
            {"data": "ParameterNamesUnique", sort: "string", type: "alt-string"},
            {"data": "PortTypeOnlyBooleanOrSIUnit", sort: "string", type: "alt-string"},
            {"data": "PortUsage", sort: "string", type: "alt-string"},
            {"data": "ReferencedSubComponentExists", sort: "string", type: "alt-string"},
            {"data": "SimpleConnectorSourceExists", sort: "string", type: "alt-string"},
            {"data": "SourceTargetNumberMatch", sort: "string", type: "alt-string"},
            {"data": "SubComponentsConnected", sort: "string", type: "alt-string"},
            {"data": "TopLevelComponentHasNoInstanceName", sort: "string", type: "alt-string"},
            {"data": "TypeParameterNamesUnique", sort: "string", type: "alt-string"},
            {"data": "AtomicComponent", sort: "string", type: "alt-string"}
        ]
    });
};

function init() {
    jQuery.fn.dataTableExt.oSort["vis-desc"] = function (x, y) {
        var res1 = $(x).hasClass('svgEye') ? 1 : -1;
        var res2 = $(y).hasClass('svgEye') ? 1 : -1;
        return res1 > res2 ? 1 : res1 >= res2 ? 0 : -1;
    };

    jQuery.fn.dataTableExt.oSort["vis-asc"] = function (x, y) {
        return jQuery.fn.dataTableExt.oSort["vis-desc"](y, x);
    }
    $("th.expandChildren-control").on('click', function () {
        window.parent.tabGrouped();
    });
    table = $(tableReference).DataTable();
    initFloatingHeader(0);
    initLogMechanic('details-control', formatLog_custom);
    initGrowMechanic('grow', 'shortLabel', 'fullLabel');
    window.parent.expandedFrameInited();
    console.log("Expanded Frame Init Complete")
}

function formatLog_custom(d) {
    // `d` is the original data object for the row
    return '<tr>' +
        '<td>Path:</td>' +
        '<td>' + d.Path + '</td>' +
        '</tr>' +
        '<tr>' +
        '<td>Log output:</td>' +
        '<td>' + d.LogOutput + '</td>' +
        '</tr>';
}

function uniqueNameFunction_custom(data) {
    var res = data['Root'] + "_" + data['Path'] + +"_" + data['Name'];
    res = res.split(' ').join('');
    res = res.replace(/\./g, "_").replace(/\(/g, "").replace(/\)/g,"")
        .replace(/\\/g, "_").replace(/\//g,"_");
    return res;
}

function columnFilter_custom(rowIdx) {
    return (table.row(rowIdx).data().Name.indexOf('Parsing failed') !== -1 ||
        table.row(rowIdx).data().Name.indexOf('Resolving failed') !== -1)
        ? true : false;
}
