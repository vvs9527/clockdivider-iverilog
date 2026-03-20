# clockdivider-iverilog

Minimal repository for the `clockdivider` RTL, its Verilog testbench, and a one-command Icarus Verilog flow.

## Files

- `clockdivider.v`: DUT
- `tb_clockdivider.v`: testbench
- `run_iverilog.ps1`: compile and run script

## Requirements

- `iverilog`
- `vvp`

## Run

```powershell
pwsh .\run_iverilog.ps1
```

Or in Windows PowerShell:

```powershell
.\run_iverilog.ps1
```

The script writes build and waveform outputs into `out/`.
