/* (c) https://github.com/MontiCore/monticore */
var infoDate = "";

function createTable_custom(files) {
    loadJSONFiles(files, {}, computeDataThenCreateTable)
}

function mergeErroredData(data) {
    if (data.length == 0) return data;
    log('Start merging');
    var mainErrorPackage = data[0];
    for (var i = 1; i < data.length; i++) {
        mainErrorPackage['ChildData'] = mainErrorPackage['ChildData'].concat(data[i]['ChildData']);
    }
    var newName = mainErrorPackage['Name'];
    mainErrorPackage['Name'] = newName.substring(0, newName.indexOf('(') + 1) + mainErrorPackage['ChildData'].length + ')';
    log('End merging');
    return [mainErrorPackage];
}

function computeDataThenCreateTable(datas) {
    var data = datas[dataFile];
    var dataParsingErrored = mergeErroredData(datas[dataParsingErroredFile]);
    var dataResolvingErrored = mergeErroredData(datas[dataResolvingErroredFile]);
    data.push(dataParsingErrored[0], dataResolvingErrored[0]);
    var info = datas[infoFile];
    data.forEach(function (datum) {
        var order = uniqueNameFunction_custom(datum);
        datum['Order'] = order;
    });
    infoDate = info['date'];
    createTable(data, info);
}

function createTable(data, info) {
    table = $(tableReference).DataTable({
        "fnInitComplete": function () {
            init();
        },
        "fixedHeader": true,
        "select": true,
        "data": data,
        "orderCellsTop": true,
        "ordering": true,
        "paging": false,
        "bInfo": false,
        "orderFixed": {"pre": [0, 'asc'], "pre": [1, 'asc']},
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
            {"data": "Order", "visible": false},
            {
                "className": "expandChildren-control",
                "data": "ChildExpansion",
                "orderable": false,
            },
            {
                "data": "Name",
                "bSortable": true,
            },
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
            {"data": "ComponentWithTypeParametersHasInstance", sort: "string", type: "alt-string"},
            {"data": "ConnectorEndPointCorrectlyQualified", sort: "string", type: "alt-string"},
            {"data": "DefaultParametersHaveCorrectOrder", sort: "string", type: "alt-string"},
            {"data": "InPortUniqueSender", sort: "string", type: "alt-string"},
            {"data": "InRosPortRosSender", sort: "string", type: "alt-string"},
            {"data": "OnlyIncomingPortIsConfig", sort: "string", type: "alt-string"},
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
            {"data": "UniquePorts", sort: "string", type: "alt-string"},
            {"data": "AtomicComponentCoCo", sort: "string", type: "alt-string"},
            {"data": "MatrixAssignmentDeclarationCheck", sort: "string", type: "alt-string"},
            {"data": "MatrixAssignmentCheck", sort: "string", type: "alt-string"},
            {"data": "DynamicComponentDynamicBodyElements", sort: "string", type: "alt-string"},
            {"data": "NoDynamicNewComponentAndPort", sort: "string", type: "alt-string"},
            {"data": "NoDynamicNewConnectsOutsideEventHandler", sort: "string", type: "alt-string"},
            {"data": "ReferencedSubComponentExistsEMAM", sort: "string", type: "alt-string"},
            {"data": "CheckLayer", sort: "string", type: "alt-string"},
            {"data": "CheckRangeOperators", sort: "string", type: "alt-string"},
            {"data": "CheckVariableName", sort: "string", type: "alt-string"},
            {"data": "CheckLayerName", sort: "string", type: "alt-string"},
            {"data": "CheckArgument", sort: "string", type: "alt-string"},
            {"data": "CheckLayerRecursion", sort: "string", type: "alt-string"},
            {"data": "CheckBehaviorName", sort: "string", type: "alt-string"}
        ]
    });
};

function init() {
    $("th.expandChildren-control").on('click', function () {
        window.parent.tabExpanded();
    });
    $('thead tr').find('th').each(function (i) {
        if ($(this).text() !== "" && ($(this).hasClass('sorting') || $(this).hasClass('sorting_asc') || $(this).hasClass('sorting_desc'))) {
            $(this).on('click', function () {
                window.parent.tabExpandedWithOrder(i + 1);
            });
        }
    });
    table = $(tableReference).DataTable();
    childControlInit('expandChildren-control', uniqueNameFunction_custom);
    initFloatingHeader(0);
    initLogMechanic('details-control', formatLog_custom);
    initGrowMechanic('grow', 'shortLabel', 'fullLabel');
    defaultExpand(2, columnFilter_custom);
    insertTimeStamp(infoDate);
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
    var res = data['Root'] + "_" + data['Path'] + "_" + data['Name'];
    res = res.split(' ').join('');
    res = res.replace(/\./g, "_").replace(/\(/g, "").replace(/\)/g, "")
        .replace(/\\/g, "_").replace(/\//g, "_");
    return res;
}

function columnFilter_custom(rowIdx) {
    return (table.row(rowIdx).data().Name.indexOf('Parsing failed') !== -1 ||
        table.row(rowIdx).data().Name.indexOf('Resolving failed') !== -1)
        ? true : false;
}
