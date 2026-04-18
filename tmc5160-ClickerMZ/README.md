# tmc5160-ClickerMZ

Basic XC32 project created with PIC32-IDE-VSCode extension.

## Project Structure

```
tmc5160-ClickerMZ/
├── srcs/
│   └── main.c       # Main application code
├── incs/            # Header files
├── objs/            # Object files (generated)
├── bins/            # Binary outputs (generated)
├── Makefile         # Build configuration
└── .vscode/
    └── tasks.json   # VS Code build tasks
```

## Build Instructions

1. **Configure Device**: Edit `Makefile` and set `DEVICE` to your target PIC32 device
2. **Configure XC32 Path**: Verify `XC32_PATH` in `Makefile` matches your installation
3. **Configure DFP Path**: DFP auto-detected and configured
4. **Build**: Press `Ctrl+Shift+B` or run `make`
5. **Flash**: Run `make flash` (configure flash tool first)



## Next Steps

- Add more source files to `srcs/` directory
- Configure device-specific settings in `main.c`
- Add peripheral initialization code
- Configure bootloader or programmer for flashing

Generated: 4/17/2026, 11:12:17 PM
