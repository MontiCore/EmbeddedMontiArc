# (c) https://github.com/MontiCore/monticore  
#Download curl
if(!(Test-Path -path ".\curl")) {
  "Getting curl"
  $file = ".\curl.zip"
  $file = [System.IO.Path]::GetFullPath($file)
  Import-Module BitsTransfer
  Start-BitsTransfer -Source $urlcurl -Destination $file

  $shell = new-object -com shell.application
  $zip = $shell.NameSpace($file)
  foreach($item in $zip.items()) {
    $shell.Namespace($build).copyhere($item)
  }
  
  mkdir "curl"
  mv ".\curl-7.59.0-win32-mingw\bin\*" "curl\"
  rm -Recurse -Force ".\curl-7.59.0-win32-mingw"
  rm -Force ".\curl.zip"
}

#Download 7zip
if(!(Test-Path -path ".\7z")) {
  "Getting 7Zip"
  
  Start-Process $curl -ArgumentList '-LJO', $url7z -NoNewWindow -Wait
  
  #extract 7z
  $file = ".\7ZipPortable.zip"
  $file = [System.IO.Path]::GetFullPath($file)
  $shell = new-object -com shell.application
  $zip = $shell.NameSpace($file)
  foreach($item in $zip.items()) {
    $shell.Namespace($build).copyhere($item)
  }
  
  mkdir "7z"
  mv ".\7ZipPortable\App\7-Zip\7*" "7z\"
  rm -Recurse -Force ".\7ZipPortable"
  rm -Force ".\7ZipPortable.zip"
}
