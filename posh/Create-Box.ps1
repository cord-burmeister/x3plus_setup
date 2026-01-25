#<#
#.Synopsis
#	Create a new Vagrant box file for an ubuntu environment with 
# preinstalled ROS 2 distribution.
#.Description
#	The Virtual machine configuration in the Hyper-V Manager 
# requires some post processing to be used as box in the vagrant 
# environmetn
#.Parameter BoxName
#    This is the name of the vagrant box which will be generated. This is also the name of the 
#    folder which contains the vagrant file which generates the default name of the VM in hyper-V
#.Parameter VmName
#    This is the name of the Hyper-V source virtual machine, when set.
#    Otherwise it will be using the VMs which starts with  'vagrant-windows_default'
#.Parameter WorkingFolder
#    This is the path to the folder in which the box will be created
#    and compressed intermediate files. 
#.Example
#	Create a box file from the Hyper-V machine server2022
# CreateVagrantBox -VmName server2022
##>

param (	
  [string] $BoxName = "jammy-humble-harmonic",
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

$FullBoxName = "marvin/$BoxName"
$VmName = "box-$BoxName"
Write-Host "Generate the box with name $FullBoxName from VMs $VmName in folder $WorkingFolder"

Set-Location $PSScriptRoot/../ubuntu-box/$VmName
& "C:\Program Files\Vagrant\bin\vagrant.exe" destroy -f 
& "C:\Program Files\Vagrant\bin\vagrant.exe" up
& "C:\Program Files\Vagrant\bin\vagrant.exe" halt

Set-Location $location

& ./CreateVagrantBox.ps1 -BoxName $BoxName -PrefixVM $VmName -WorkingFolder $WorkingFolder
& ./PublishBoxFile.ps1 -BoxName $BoxName -WorkingFolder $WorkingFolder -Comment $comment 

# Comment this, when you want the resulting box for debugging purposes. 
Set-Location $PSScriptRoot/../ubuntu-box/$VmName
& "C:\Program Files\Vagrant\bin\vagrant.exe" destroy -f 
Set-Location $location
