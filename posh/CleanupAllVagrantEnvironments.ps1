# Recursively search for directories containing a Vagrantfile
# and run `vagrant destroy -f` in each of them.

$root = "$PSScriptRoot\.."   # adjust as needed

# Returns $true if the current PowerShell session is elevated
$IsAdmin = ([Security.Principal.WindowsPrincipal] `
    [Security.Principal.WindowsIdentity]::GetCurrent()
).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)

if ($IsAdmin) {
    Write-Host "Running with administrative privileges." -ForegroundColor Green
} else {
    Write-Host "NOT running as administrator." -ForegroundColor Red
    Exit 0
}


Write-Host "Cleanup Vagrant environments in:" $root -ForegroundColor Cyan

Get-ChildItem -Path $root -Recurse -Directory |
    Where-Object { Test-Path (Join-Path $_.FullName "Vagrantfile") } |
    ForEach-Object {
        Write-Host "Found Vagrant environment in:" $_.FullName -ForegroundColor Cyan

        Push-Location $_.FullName
        try {
            Write-Host "Destroying..." -ForegroundColor Yellow
            vagrant destroy -f
        }
        catch {
            Write-Warning "Failed to destroy VM in $($_.FullName): $_"
        }
        finally {
            Pop-Location
        }
    }

Write-Host "Cleanup complete." -ForegroundColor Green
