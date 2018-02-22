function loadJSON(file,callback) {   
  var xobj = new XMLHttpRequest();
  xobj.overrideMimeType('application/json');
  xobj.open('GET', file, true); 
  xobj.onreadystatechange = function () {
    if (xobj.readyState == 4 && xobj.status == '200') {
    // Required use of an anonymous callback as .open will NOT return a value but simply returns undefined in asynchronous mode
      callback(xobj.responseText);
     }
  };
  xobj.send(null);  
}

function createTable(data) {
  data = JSON.parse(data);
  table = $('#my-table').DataTable( {
      fixedHeader: true,
      "data" : data,
      "language": {
        "loadingRecords": "Please wait - loading..."
      },
      "processing": true,
      "orderCellsTop": true,
      "aaSorting": [5,'asc'],
      "ordering": true,
      "paging" : false,
      "bInfo": false,
      "orderFixed": [0, 'asc'],
      "select": true,
      "rowGroup": {
        dataSrc: "Root",
        startRender: function ( rows, group ) {
            var valid = rows
                .data()
                .pluck('Valid')
                .reduce( function (a, b) {
                    if(b==undefined){
                        return 0;
                    }
                    if (b.toLowerCase().indexOf("cross") >= 0){
                        return a + 0;
                    } else {
                        return a + 1;
                    }
                }, 0);

            var count = rows.count();
            return group +' ('+count+')&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; valid: ' + valid + '&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; invalid: ' + (count-valid);
        },
        endRender: null //function( rows, group ) {return group;}
      },
      "aoColumns": [
          { "data": "Root", visible: false},
          { 
              "className": 'grow',
              "data": "Name", 
              "sType": "growDiv",
              "bSortable": true
              
          },
          { "data": "OnlineIDE", "orderable": false },
          {
              "className":      'details-control',
              "orderable":      false,
              "data":           null,
              "defaultContent": ''
          },
          { "data": "FileType", sort:"string" },
          { "data": "Valid", sort:"string", type:"alt-string" },
          { "data": "Parse", sort:"string", type:"alt-string" },
          { "data": "Resolve", sort:"string", type:"alt-string" },
          { "data": "ComponentCapitalized", sort:"string", type:"alt-string" },
          { "data": "ComponentInstanceNamesUnique", sort:"string", type:"alt-string" },
          { "data": "ComponentWithTypeParametersHasInstance", sort:"string", type:"alt-string" },
          { "data": "ConnectorEndPointCorrectlyQualified", sort:"string", type:"alt-string" },
          { "data": "DefaultParametersHaveCorrectOrder", sort:"string", type:"alt-string" },
          { "data": "InPortUniqueSender", sort:"string", type:"alt-string" },
          { "data": "PackageLowerCase", sort:"string", type:"alt-string" },
          { "data": "ParameterNamesUnique", sort:"string", type:"alt-string" },
          { "data": "PortTypeOnlyBooleanOrSIUnit", sort:"string", type:"alt-string" },
          { "data": "PortUsage", sort:"string", type:"alt-string" },
          { "data": "ReferencedSubComponentExists", sort:"string", type:"alt-string" },
          { "data": "SimpleConnectorSourceExists", sort:"string", type:"alt-string" },
          { "data": "SourceTargetNumberMatch", sort:"string", type:"alt-string" },
          { "data": "SubComponentsConnected", sort:"string", type:"alt-string" },
          { "data": "TopLevelComponentHasNoInstanceName", sort:"string", type:"alt-string" },
          { "data": "TypeParameterNamesUnique", sort:"string", type:"alt-string" },
          { "data": "AtomicComponent", sort:"string", type:"alt-string" }
      ],
      "order": [[1, 'asc']]
  } );      
};
  
/* Formatting function for row details - modify as you need */
function format ( d ) {
    // `d` is the original data object for the row
    return '<table cellpadding="5" cellspacing="0" border="0" style="padding-left:50px;">'+
        '<tr>'+
            '<td>Log output:</td>'+
            '<td>'+d.LogOutput+'</td>'+
        '</tr>'+
    '</table>';
}

function adjustHeader() {
  $('#my-table thead tr:eq(1) th').each( function(i) {
    $(this).text( '' );
  });
}

function adjustFloatingHeader(content) {
  var first = true;
  $('.fixedHeader-floating thead tr:eq(1) th').each( function(i) {
    if(first) {
      first = false;
      $(this).html( content );
    } else {
        $(this).text( '' );
    }
  });
}

function getStringFromGrow(x) {
  str1 = $(x).find('.noSVGhidden').text().toLowerCase();
  str2= $(x).find('.sVGhidden').text().toLowerCase();
  return str1 == "" ? str2 : str1;
}
      
$(document).ready(function() {
  jQuery.fn.dataTableExt.oSort["growDiv-desc"] = function (x, y) {
    return getStringFromGrow(x) < getStringFromGrow(y);
  };
       
  jQuery.fn.dataTableExt.oSort["growDiv-asc"] = function (x, y) {
    return getStringFromGrow(x) > getStringFromGrow(y);
  }    

    
  $('#my-table thead tr').clone(true).appendTo( '#my-table thead' );
  $('#my-table thead tr:eq(1)').addClass('group');
  adjustHeader();
  
  loadJSON("data/data.json", createTable);
   
  // Add event listener for opening and closing details
  $('#my-table tbody').on('click', 'td.details-control', function () {
      var tr = $(this).closest('tr');
      var row = table.row( tr );

      if ( row.child.isShown() ) {
          // This row is already open - close it
          row.child.hide();
          tr.removeClass('shown');
      }
      else {
          // Open this row
          row.child( format(row.data()) ).show();
          tr.addClass('shown');
      }
  } );
  
  var init = false;
  var lastIndex = -1;
  $(document).scroll( function() {
    var currentGroup = "EmbeddedMontiArc";
    var index = 0;
    var lastFoundIndex = -1;
    var lastFound = "";
    $('#my-table tbody .group td').each(function() {
      
      var docViewTop = $(window).scrollTop();
      var elemTop = $(this).offset().top;
      var elemBottom = elemTop + $(this).height();
      
      if ( elemBottom < docViewTop || elemTop < docViewTop) {
        lastFound = $(this).html();
        lastFoundIndex = index;
      }
      index++;
    });
    
    if( lastFoundIndex != lastIndex || init == false) {
      lastIndex = lastFoundIndex;
      if ( $('.fixedHeader-floating')[0] ) {
        adjustFloatingHeader(lastFound);
        init = true;
      }
    }
    
    if ( !$('.fixedHeader-floating')[0] ) {
      init = false;
    }
  });
  
  $('.grow').mouseover( function(){
    $(this).find('.shortLabel').hide();
    $(this).find('.fullLabel').show();
  });
  $('.grow').mouseout( function(){
    $(this).find('.shortLabel').show();
    $(this).find('.fullLabel').hide();
  });
  
  $('#my-table tbody').on('mouseenter', 'td.grow', function() {
    $(this).find('.shortLabel').css('display', 'none');
    $(this).find('.fullLabel').css('display', 'block');
  } );
  $('#my-table tbody').on('mouseleave', 'td.grow', function() {
    $(this).find('.shortLabel').css('display', 'block');
    $(this).find('.fullLabel').css('display', 'none');
  } );
} );
