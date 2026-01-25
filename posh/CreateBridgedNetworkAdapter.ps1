# Name of the virtual switch you want to create
$SwitchName = "External-Auto"

# Detect the active physical network adapter
# Criteria:
#   - Status = Up
#   - Not a virtual adapter
#   - Has a default gateway (meaning it's actually used for internet/LAN)
$ActiveNIC = Get-NetIPConfiguration |
    Where-Object { $_.IPv4DefaultGateway -ne $null } |
    Select-Object -First 1

if (-not $ActiveNIC) {
    Write-Host "No active network adapter with a default gateway was found."
    exit 1
}

$AdapterName = $ActiveNIC.InterfaceAlias
Write-Host "Detected active adapter: $AdapterName"

# Check if the switch already exists
if (Get-VMSwitch -Name $SwitchName -ErrorAction SilentlyContinue) {
    Write-Host "A virtual switch named '$SwitchName' already exists."
}
else {
    Write-Host "Creating external virtual switch '$SwitchName' on adapter '$AdapterName'..."
    New-VMSwitch -Name $SwitchName -NetAdapterName $AdapterName -AllowManagementOS $true
    Write-Host "Virtual switch created successfully."
}
