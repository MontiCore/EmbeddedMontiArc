function Download-File($url, $output) {
    $request = [System.Net.WebRequest]::Create("$url")
    $request.ContentType='application/json; charset=utf-8'
    $request.GetResponse()

    try
    {
        $requestStream = $request.GetRequestStream()
        $streamWriter = New-Object System.IO.StreamWriter($requestStream)
        $streamWriter.Write($body)
    }
    finally
    {
        if ($null -ne $streamWriter) { $streamWriter.Dispose() }
        if ($null -ne $requestStream) { $requestStream.Dispose() }
    }

    $response = $request.GetResponse()
    $responseStream = $response.GetResponseStream()
    $readStream = New-Object System.IO.StreamReader $responseStream
    $data=$readStream.ReadToEnd()
    $data | Out-File $output -Force
}


# download new dependencies if dependencies.txt has changed
if(!(Test-Path -path $dl)) {
    New-Item -ItemType directory -Path $dl
}

Set-Location -Path $dl
"Downloading dependencies"
        
foreach($line in Get-Content "..\dependencies.txt") {

    # if line beginns with ** extract link
    if($line -match '^\*\*.+'){
        $link = $line.Remove(0,3)

        # extract filename
        if($link -match '[a-zA-Z0-9\-\.]+\.zip'){
            $filename = $matches[0]
    
            # if file does not exist, download
            if(!(Test-Path -path $filename)) {
                "Downloading " + $filename + " from: " + $link
                Download-File($link, $filename)
            }
        }
    }
} 

Set-Location -Path "..\"

#Download 7zip
if(!(Test-Path -path ".\7ZipPortable") && !(Test-Path -path ".\7ZipPortable.zip")) {
  "Getting 7Zip"
  $url = "https://www.7-zip.org/a/7za920.zip"
  $file = ".\7z.zip"
  Download-File($url, $file)
}
#Extraxting 7z
$shell = new-object -com shell.application
$zip = $shell.NameSpace(“.\7z.zip”)
foreach($item in $zip.items()) {
    $shell.Namespace(“.\”).copyhere($item)
}


