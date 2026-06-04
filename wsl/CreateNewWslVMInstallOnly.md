# CreateNewWslVMInstallOnly.ps1

Creates a new WSL distro instance by calling `wsl --install` (install-only flow) and validating that the distro was created successfully.

## Purpose

This script automates WSL distro installation with a unique, user-friendly instance name.

It is useful when you want multiple isolated WSL environments on one Windows machine (for example, one per project or ROS setup).

## Script Location

- `wsl/CreateNewWslVMInstallOnly.ps1`

## Requirements

- Windows with WSL available (`wsl.exe` in `PATH`)
- PowerShell 5.1+ or PowerShell 7+
- Elevated terminal if your WSL setup requires admin permissions for install

## Parameters

### `-Distro` (mandatory)

- Type: `string`
- Example values: `Ubuntu-22.04`, `Ubuntu-24.04`
- Logical default in script: `Ubuntu-22.04`

Notes:

- The parameter is marked mandatory, so PowerShell prompts if omitted.
- The provided value is passed to `wsl.exe --install --distribution <value>`.

### `-NamePrefix` (optional)

- Type: `string`
- If omitted, script derives: `<Distro>-vm`

The script ensures uniqueness:

- First choice: `<NamePrefix>`
- If already used, suffixes: `<NamePrefix>-01`, `<NamePrefix>-02`, ...

## What the Script Does

1. Enables strict mode and stop-on-error behavior.
2. Verifies `wsl.exe` exists.
3. Reads currently installed distros (`wsl --list --quiet`).
4. Computes a unique target name.
5. Runs installation.
6. Re-reads installed distros and verifies the expected distro appears.
7. Prints a launch command.

## `ShouldProcess` Support

The script uses `[CmdletBinding(SupportsShouldProcess = $true)]`, so you can run with `-WhatIf` for a dry run:

```powershell
./CreateNewWslVMInstallOnly.ps1 -Distro Ubuntu-22.04 -NamePrefix dev-humble -WhatIf
```

## Examples

Install Ubuntu 22.04 with an explicit prefix:

```powershell
./CreateNewWslVMInstallOnly.ps1 -Distro Ubuntu-22.04 -NamePrefix x3plus-humble
```

Install Ubuntu 24.04 with auto-generated name prefix:

```powershell
./CreateNewWslVMInstallOnly.ps1 -Distro Ubuntu-24.04
```

If `Ubuntu-24.04-vm` already exists, the script may choose `Ubuntu-24.04-vm-01`.

## Expected Output

Success messages are similar to:

```text
Installing 'Ubuntu-22.04' as 'x3plus-humble'...
Created WSL VM 'x3plus-humble' from distro 'Ubuntu-22.04' using install-only flow.
Start it with: wsl -d x3plus-humble
```
