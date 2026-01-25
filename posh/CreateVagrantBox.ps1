#<#
#.Synopsis
#	Create a Vagrant box file out of a Hyper-V machine
#.Description
#	The Virtual machine configuration in the Hyper-V Manager 
# requires some post processing to be used as box in the vagrant 
# environmetn
#.Parameter BoxName
#    This is the name of the vagrant box which will be generated
#.Parameter PrefixVM
#    This is the prefix of the Hyper-V source virtual machine, when set.
#.Parameter WorkingFolder
#    This is the path to the folder in which the box will be created
#    and compressed intermediate files. 
#.Example
#	Create a box file from the Hyper-V machine server2022
# CreateVagrantBox -VmName server2022
##>

param (	
  [string] $BoxName = "ros2humble",
  [string] $PrefixVM = "ubuntu-box-vagrant",
  [string] $WorkingFolder = "c:\work"
  )




$MyJsonVariable = @"
{
  "provider": "hyperv"
}
"@

$VmName = ""

$a =  Get-VM |  Where-Object {$_.Name -match "$PrefixVM"}
if ($a.Count -eq 1)
{
  $VmName = $a[0].Name
}
else {
  Write-Error ("ERROR: The prefix $PrefixVM is not unique. Please specify the name with the VmName parameter.")
  Exit 1
}

$FullBoxName = "marvin/$BoxName"

Write-Host "Generate the box with name $FullBoxName from VMs $VmName in folder $WorkingFolder"

if (Test-Path $WorkingFolder\$VmName -PathType Container)
{
    Remove-Item $WorkingFolder\$VmName -Recurse -Force
}

if (Test-Path $WorkingFolder\$BoxName -PathType Container)
{
    Remove-Item $WorkingFolder\$BoxName -Recurse -Force
}

New-Item $WorkingFolder\$BoxName -ItemType Directory -Force

Export-VM -Name $VmName -Path "$WorkingFolder"
Remove-Item -Path "$WorkingFolder\$VmName\Snapshots" -Recurse -Force -ErrorAction Ignore

$Utf8NoBomEncoding = New-Object System.Text.UTF8Encoding $False
[System.IO.File]::WriteAllLines("$WorkingFolder\$VmName\metadata.json", $MyJsonVariable, $Utf8NoBomEncoding)

7z a -r $WorkingFolder\$BoxName\$BoxName.tar $WorkingFolder\$VmName\*.*
7z a -mmt9 $WorkingFolder\$BoxName\$BoxName.tar.gzip $WorkingFolder\$BoxName\$BoxName.tar
Rename-Item $WorkingFolder\$BoxName\$BoxName.tar.gzip $WorkingFolder\$BoxName\$BoxName.box
Remove-Item $WorkingFolder\$BoxName\$BoxName.tar

$vmSize = "{0:N2} GB" -f ((Get-ChildItem $WorkingFolder\$VmName -Recurse -Exclude "*.box" | Measure-Object -Property Length -Sum -ErrorAction Stop).Sum / 1GB)
$boxSize = "{0:N2} GB" -f ((Get-ChildItem $WorkingFolder\$BoxName -Recurse -Include "*.box" | Measure-Object -Property Length -Sum -ErrorAction Stop).Sum / 1GB)

# if (Test-Path "$WorkingFolder\Virtual Machines" -PathType Container)
# {
#     Remove-Item "$WorkingFolder\Virtual Machines" -Recurse -Force
# }
# if (Test-Path "$WorkingFolder\Virtual Hard Disks" -PathType Container)
# {
#     Remove-Item "$WorkingFolder\Virtual Hard Disks" -Recurse -Force
# }

Write-Host "Created a box file. Box Size $boxSize. Virtual Machine Size $vmSize "
Write-Host "Do add the box manually use:"
Write-Host vagrant box add --name $FullBoxName "$WorkingFolder\$BoxName\$BoxName.box"