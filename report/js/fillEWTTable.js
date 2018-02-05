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
      "order": [[ 5, "desc" ]],
      "ordering": true,
      "paging" : false,
      "columns": [
          {
              "data": "Path", 
              sort: "string" 
          },
          {
              "data": "NameEndsWithTest",
              sort:"string", type:"alt-string"
          }
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
      
$(document).ready(function() {

  loadJSON("data/dataEWT.json", createTable);
   
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