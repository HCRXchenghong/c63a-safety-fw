$real = 'C:\Users\HCRXc\arm-gnu-toolchain-14.2.rel1\bin\arm-none-eabi-gcc.exe'
$forward = @()

foreach ($arg in $args) {
    if ($arg -ceq '-fcyclomatic-complexity') {
        continue
    }

    $forward += $arg
}

& $real @forward
exit $LASTEXITCODE
