function createTable(file, infoFile) {
    loadInfo(file, infoFile, createTable_);
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
            loadJSON(file, callback, res);
        }
    };
    xobj.send(null);
}

function loadJSON(file, callback, info) {
    var xobj = new XMLHttpRequest();
    xobj.overrideMimeType('application/json');
    xobj.open('GET', file, true);
    xobj.onreadystatechange = function () {
        if (xobj.readyState == 4 && xobj.status == '200') {
            // Required use of an anonymous callback as .open will NOT return a value but simply returns undefined in asynchronous mode
            callback(xobj.responseText, info);
        }
    };
    xobj.send(null);
}

function createTable_(data, info) {
    data = JSON.parse(data);
    table = $(tableReference).DataTable({
        fixedHeader: true,
        "data": data,
        "language": {
            "loadingRecords": "Please wait - loading..."
        },
        "processing": true,
        "orderCellsTop": true,
        "aaSorting": [5, 'asc'],
        "ordering": true,
        "paging": false,
        "bInfo": false,
        "orderFixed": {
            "pre": [0, 'asc'],
            "pre": [1, 'asc']
        },
        "select": true,
        "rowGroup": {
            "dataSrc": "Root",
            "startRender": function (rows, group) {
                var count = info[group]['Number'];
                var valid = info[group]['Valid'];
                var invalid = info[group]['Invalid'];
                return group + ' (' + count + ')&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; valid: ' + valid + '&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; invalid: ' + invalid;
            },
            "endRender": null //function( rows, group ) {return group;}
        },
        "bAutoWidth": false,
        "aoColumns": [
            {"data": "Root", "visible": false},
            {"data": "Order", "visible": false},
            {
                "className": "expandChildren-control",
                "data": "ChildExpansion",
                "orderable": false,
            },
            {
                "className": 'groupName',
                "data": "Name",
                "sType": "groupName",
                "bSortable": true,
                "sWidth": "50%"
            },
            {
                "data": "OnlineIDE",
                "className": "onlineIDE",
                "orderable": false,
                "visible": ide
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
        ],
        "fnInitComplete": function(oSettings, json) {
            init();
        }
    });
};

function init() {
    childControlInit();
    initFloatingHeader();
    initLogMechanic('details-control', formatLog1);
    initGrowMechanic('grow', 'shortLabel', 'fullLabel');
}

removeChildRows = function (tr, row, data) {
    const table = $(tableReference).DataTable();
    var name = data.Order.replace(/\./g, "_").replace(/\(/, "").replace(/\)/,"").split(' ').join('');

    $('.' + name + '_Child').each(function () {
        var childTr = $(this).closest('tr');
        var childRow = table.row(childTr);
        table.row(childRow).remove();
    });

    $(tr).removeClass('childrenShown');
    $(tr).addClass('childrenNotShown');
}

expandRow = function (tr, row, data) {

    if (data.ChildData.length != 0) {


        const table = $(tableReference).DataTable();
        var name = data.Order.replace(/\./g, "_").replace(/\(/, "").replace(/\)/,"").split(' ').join('');

        if ($(tr).hasClass('childrenShown')) {
            removeChildRows(tr, row, data);
        } else {
            for (var i = 0; i < data.ChildData.length; i++) {
                const tmpRow = table.row.add(data.ChildData[i]).node();

                var childRow = table.row(tmpRow);
                childRow.data()['Order'] = data['Order'] + "_Child";

                $(tmpRow).addClass('childRow1')
                $(tmpRow).addClass((name + "_Child"));
                $(tmpRow).removeClass('childrenNotShown');
            }
            $(tr).removeClass('childrenNotShown');
            $(tr).addClass('childrenShown');
        }
        table.draw();
    }

};

function childControlInit() {
    var table = $(tableReference).DataTable();

    $(tableReference + ' tbody').on('click', 'td.expandChildren-control', function () {
        var tr = $(this).closest('tr');
        var row = table.row(tr);

        expandRow(tr, row, row.data());
    });


    var indexes = table.rows().eq(0).filter(function (rowIdx) {
        var res = (table.cell(rowIdx, 3).data().indexOf('Parsing failed') !== -1 || table.cell(rowIdx, 3).data().indexOf('Resolving failed') !== -1) ? true : false;
        return res;
    });

    table.rows(indexes).eq(0).map(function (rowIdx) {
        table.cell(rowIdx, 2).nodes().to$().click();
    });
}

$(document).ready(function () {
    initHeader();

    createTable(dataFile, infoFile);
});
