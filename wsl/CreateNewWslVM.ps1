[CmdletBinding(SupportsShouldProcess = $true)]
param(
	[Parameter(Mandatory = $true)]
	[string] $SourceDistro,

	[switch] $UseInstall,

	[string] $NamePrefix,

	[string] $InstallRoot = "$env:USERPROFILE\WSL",

	[string] $TempRoot = "$env:TEMP"
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

if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) {
	throw "wsl.exe not found. Run this script on Windows with WSL installed."
}

$existingDistros = @(Get-InstalledDistros)

if ($existingDistros -notcontains $SourceDistro) {
	if (-not $UseInstall) {
		throw "Source distro '$SourceDistro' is not installed. Use -UseInstall to install it first."
	}

	if ($PSCmdlet.ShouldProcess($SourceDistro, "Install source distro using wsl --install")) {
		Write-Host "Installing '$SourceDistro' with wsl --install..."
		& wsl.exe --install --distribution $SourceDistro
		if ($LASTEXITCODE -ne 0) {
			throw "Install of '$SourceDistro' failed."
		}
	}

	$existingDistros = @(Get-InstalledDistros)
	if ($existingDistros -notcontains $SourceDistro) {
		throw "Distro '$SourceDistro' was not detected after install."
	}
}

if ([string]::IsNullOrWhiteSpace($NamePrefix)) {
	$NamePrefix = "$SourceDistro-clone"
}

$targetDistro = Get-UniqueDistroName -BaseName $NamePrefix -ExistingNames $existingDistros
$targetInstallPath = Join-Path -Path $InstallRoot -ChildPath $targetDistro

if (-not (Test-Path -Path $InstallRoot -PathType Container)) {
	if ($PSCmdlet.ShouldProcess($InstallRoot, "Create install root directory")) {
		New-Item -Path $InstallRoot -ItemType Directory -Force | Out-Null
	}
}

if (Test-Path -Path $targetInstallPath) {
	throw "Install path '$targetInstallPath' already exists. Use another prefix or remove the folder."
}

$exportFile = Join-Path -Path $TempRoot -ChildPath ("wsl-export-{0}.tar" -f [guid]::NewGuid().ToString("N"))

try {
	if ($PSCmdlet.ShouldProcess($SourceDistro, "Export source distro to '$exportFile'")) {
		Write-Host "Exporting '$SourceDistro'..."
		& wsl.exe --export $SourceDistro $exportFile
		if ($LASTEXITCODE -ne 0) {
			throw "Export of '$SourceDistro' failed."
		}
	}

	if ($PSCmdlet.ShouldProcess($targetDistro, "Import distro to '$targetInstallPath'")) {
		Write-Host "Importing as '$targetDistro'..."
		& wsl.exe --import $targetDistro $targetInstallPath $exportFile --version 2
		if ($LASTEXITCODE -ne 0) {
			throw "Import to '$targetDistro' failed."
		}
	}

	Write-Host "Created WSL VM '$targetDistro' from '$SourceDistro'."
	Write-Host "Start it with: wsl -d $targetDistro"
}
catch {
	# Clean up partially created distro on import errors.
	$updatedDistros = @(Get-InstalledDistros)
	if ($updatedDistros -contains $targetDistro) {
		Write-Warning "Removing partially created distro '$targetDistro'."
		& wsl.exe --unregister $targetDistro | Out-Null
	}

	throw
}
finally {
	if (Test-Path -Path $exportFile -PathType Leaf) {
		Remove-Item -Path $exportFile -Force
	}
}
