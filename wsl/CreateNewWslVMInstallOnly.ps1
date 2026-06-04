[CmdletBinding(SupportsShouldProcess = $true)]
param(
	[Parameter(Mandatory = $true)]
	[string] $Distro = "Ubuntu-22.04",
	[string] $NamePrefix
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Get-InstalledDistros {
	$distros = & wsl.exe --list --quiet 2>$null
	if ($LASTEXITCODE -ne 0) {
		throw "Failed to query installed WSL distros. Ensure WSL is installed and available."
	}

	return $distros |
		ForEach-Object { $_.Trim() } |
		Where-Object { $_ -ne "" }
}

function Get-UniqueDistroName {
	param(
		[Parameter(Mandatory = $true)]
		[string] $BaseName,
		[Parameter(Mandatory = $true)]
		[string[]] $ExistingNames
	)

	if ($ExistingNames -notcontains $BaseName) {
		return $BaseName
	}

	$index = 1
	while ($true) {
		$candidate = "{0}-{1:00}" -f $BaseName, $index
		if ($ExistingNames -notcontains $candidate) {
			return $candidate
		}
		$index++
	}
}

function Test-InstallNameSupport {
	$installHelp = & wsl.exe --install --help 2>$null
	if ($LASTEXITCODE -ne 0) {
		return $false
	}

	return $installHelp -match "--name"
}

if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) {
	throw "wsl.exe not found. Run this script on Windows with WSL installed."
}

$existingDistros = @(Get-InstalledDistros)

if ([string]::IsNullOrWhiteSpace($NamePrefix)) {
	$NamePrefix = "$Distro-vm"
}

$targetName = Get-UniqueDistroName -BaseName $NamePrefix -ExistingNames $existingDistros
#$hasNameSupport = Test-InstallNameSupport
$hasNameSupport = $True

if ($hasNameSupport) {
	if ($PSCmdlet.ShouldProcess($targetName, "Install distro '$Distro' using wsl --install --name")) {
		Write-Host "Installing '$Distro' as '$targetName'..."
		& wsl.exe --install --distribution $Distro --name $targetName
		if ($LASTEXITCODE -ne 0) {
			throw "Install failed for distro '$Distro' as '$targetName'."
		}
	}

	$updatedDistros = @(Get-InstalledDistros)
	if ($updatedDistros -notcontains $targetName) {
		throw "Install completed but '$targetName' was not found in installed distros."
	}

	Write-Host "Created WSL VM '$targetName' from distro '$Distro' using install-only flow."
	Write-Host "Start it with: wsl -d $targetName"
}
else {
	if ($existingDistros -contains $Distro) {
		throw "This WSL version does not support '--name' on install and distro '$Distro' is already installed."
	}

	if ($PSCmdlet.ShouldProcess($Distro, "Install distro using wsl --install")) {
		Write-Host "Installing '$Distro' with wsl --install..."
		& wsl.exe --install --distribution $Distro
		if ($LASTEXITCODE -ne 0) {
			throw "Install failed for distro '$Distro'."
		}
	}

	$updatedDistros = @(Get-InstalledDistros)
	if ($updatedDistros -notcontains $Distro) {
		throw "Install completed but '$Distro' was not found in installed distros."
	}

	Write-Warning "Install succeeded, but this WSL version cannot set a custom name during install."
	Write-Host "Created WSL VM '$Distro' from distro '$Distro' using install-only flow."
	Write-Host "Start it with: wsl -d $Distro"
}
