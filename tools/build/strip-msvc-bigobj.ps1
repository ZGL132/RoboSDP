param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$Compiler,

    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Arguments
)

$filteredArguments = @()
foreach ($argument in $Arguments)
{
    if ($argument -eq '/bigobj')
    {
        continue
    }

    $filteredArguments += $argument
}

& $Compiler @filteredArguments
exit $LASTEXITCODE
