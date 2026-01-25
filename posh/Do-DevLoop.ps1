
#<#
#.Synopsis
#	Do the complete round trip for jammy humble harmonic  .
#.Description
#	Generates a new box, publishes it and restart a new development environment 
##>


param (	
  [string] $Comment = "",  
  [string] $WorkingFolder = "c:\work"
  )

if (!$env:VAGRANTBOXSHARE)
{
    Write-Error "VAGRANTBOXSHARE environment variable is not set. Add the folder share information."
    Exit 12
}
$location = Get-Location


if ($Comment)
{
    $comment = $Comment
}
else {
    $comment = "Generated box on $(Get-Date)"
}

& ./Create-Jammy-Humble-Harmonic-Box.ps1 -Comment $comment

Set-Location $PSScriptRoot/../ubuntu-dev/default

& "C:\Program Files\Vagrant\bin\vagrant.exe" destroy -f 
& "C:\Program Files\Vagrant\bin\vagrant.exe" box prune -f
& "C:\Program Files\Vagrant\bin\vagrant.exe" box remove marvin/jammy-humble-harmonic 
& "C:\Program Files\Vagrant\bin\vagrant.exe" up


Set-Location $location