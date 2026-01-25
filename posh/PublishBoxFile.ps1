#<#
#.Synopsis
#	Create a Vagrant box file out of a Hyper-V machine
#.Description
#	The Virtual machine configuration in the Hyper-V Manager 
# requires some post processing to be used as box in the vagrant 
# environmetn
#.Parameter BoxName
#    This is the name of the box file and the vagrant box name which is prefixed 
#    with marvin/
#.Parameter Comment
#    This is the path to the folder in which the box will be created
#    and compressed intermediate files. 
#.Example
#	Create a box file from the Hyper-V machine server2022
# CreateVagrantBox BoxName server2022
##>

param (	[string] $BoxName = "ros2humble",
[string] $Comment = "Update on 2024-02-17",
[string] $WorkingFolder = "C:\work" 
)

if (!$env:VAGRANTBOXSHARE)
{
    Write-Error "VAGRANTBOXSHARE environment variable is not set. Add the folder share information."
    Exit 12
}


Import-Module BitsTransfer
# ======================================================================

$vmName = $BoxName

# This is for working on development computer 
$workingFolder = $WorkingFolder
# This is for debugging purposes 
#$publishingFolder = "F:\work\boxregistry"
$publishingFolder = $env:VAGRANTBOXSHARE
$comment = $Comment
# ======================================================================

[string] $boxMetaDataFilename =   "$publishingFolder\$vmName" +"Metadata.json"
[string] $boxSourceFilename =   "$workingFolder\$vmName\$vmName.box"
[string] $boxDestinationFilename =   "$publishingFolder\$vmName\$vmName.box"
[string] $boxDestinationUrl = $boxDestinationFilename.Replace("\", "/").Replace("//", "file:////")


$MyMetaDataFile = @"
{
    "name":  "marvin/$vmName",
    "description":  "This box contains $vmName.",
    "versions":  [
                     {
                         "version":  "0.0.1",
                         "description_markdown":  "Initial Base box",
                         "providers":  [
                                           {
                                               "name":  "hyperv",
                                               "architecture": "amd64",
                                               "url":  "$boxDestinationUrl"
                                           }
                                       ]
                     }
                 ]
}
"@

Write-Host $boxMetaDataFilename


if (-Not (Test-Path -Path $boxSourceFilename))
{
    Write-Host "Box Source file can not be found  " $boxSourceFilename
    Exit
}

if (-Not (Test-Path -Path "$publishingFolder\$vmName" -PathType Container))
{
    New-Item -Path "$publishingFolder\$vmName" -ItemType Directory
}


Start-BitsTransfer -Source $boxSourceFilename -Destination $boxDestinationFilename -Description "Transfer box file to share" -DisplayName "Publish box"
$newVersion = "unknown"
$Utf8NoBomEncoding = New-Object System.Text.UTF8Encoding $False
# Now adjust the version number of the box Metadata 
if (Test-Path -Path $boxMetaDataFilename)
{
    Write-Host "Found " $boxMetaDataFilename
    [string] $content = Get-Content -Raw -Path $boxMetaDataFilename
    [PSCustomObject] $jsonObject = $content | ConvertFrom-Json

    $version = $jsonObject.versions.version

    $ParsedVersion = $null
    if ([System.Version]::TryParse($version , [ref] $ParsedVersion ))
    {
        Write-Host "Found valid " $version

        $newVersion = $ParsedVersion.Major.ToString() + "." + ($ParsedVersion.Minor + 1).ToString() + "." + $ParsedVersion.Build.ToString()
        Write-Host "Make  " $newVersion
        $jsonObject.versions[0].version = $newVersion

        $MyJsonVariable = $jsonObject | ConvertTo-Json -Depth 100 
        
        [System.IO.File]::WriteAllLines("$boxMetaDataFilename", $MyJsonVariable, $Utf8NoBomEncoding)
    }
    else {
        Write-Host "Can not find version in meta file, so no change " 
    }
}
else {
    [System.IO.File]::WriteAllLines("$boxMetaDataFilename", $MyMetaDataFile, $Utf8NoBomEncoding)
}

$boxSize = "{0:N2} GB" -f ((Get-ChildItem $workingFolder\$vmName -Recurse -Include "*.box" | Measure-Object -Property Length -Sum -ErrorAction Stop).Sum / 1GB) 
#$vmSize  = "{0:N2} GB" -f ((Get-ChildItem $workingFolder\$vmName -Recurse -Exclude "*.box" | Measure-Object -Property Length -Sum -ErrorAction Stop).Sum / 1GB) 

$headerLine    = "| Box name              | Box Size |  Version | Date      | Changes |"
$separatorLine = "|-----------------------|----------|----------|-----------|---------|"
$changeLogFile = [System.IO.Path]::Combine($publishingFolder, "Changelog.md")

$date = Get-Date

$dateString = ""
$dateString = $dateString + $date.Day
$dateString = $dateString + "."
$dateString = $dateString + $date.Month
$dateString = $dateString + "."
$dateString = $dateString + $date.Year



$newChangeEntry = "| $vmName          | $boxSize   | $newVersion   | $dateString | $comment |"

if (Test-Path -Path $changeLogFile)
{
    $content = Get-Content -Path $changeLogFile | Out-String
    $content = $content.Replace($separatorLine, $separatorLine + "`n" + $newChangeEntry )
    Set-Content -Path $changeLogFile $content
}
else {
    [System.IO.File]::WriteAllText($changeLogFile, "# Changelog for base box images`n")
    [System.IO.File]::AppendAllText($changeLogFile, "`n")
    [System.IO.File]::AppendAllText($changeLogFile, $headerLine + "`n")
    [System.IO.File]::AppendAllText($changeLogFile, $separatorLine + "`n")
    [System.IO.File]::AppendAllText($changeLogFile, $newChangeEntry + "`n")
}

