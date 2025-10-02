El archivo start.s define el arranque en baremetal: inicializa el stack pointer (sp), limpia la sección .bss, opcionalmente copia .data, y transfiere el control a main. También escribe el registro mtvec (vector de trampas) para excepciones. El linker script sitúa la RAM en 0x80000000 (QEMU virt), define símbolos como _stack_top, _sbss, _ebss y _end, y establece ENTRY(_start) para que la ejecución inicie en Start.S.

El Makefile automatiza la compilación usando el toolchain riscv-none-elf-, habilita la extensión Zicsr (-march=rv32im_zicsr), y genera los binarios (.elf, .lst, .bin). Incluye objetivos prácticos como:

make            # compila el proyecto
make run        # ejecuta en QEMU
make gdbserver  # inicia QEMU en modo depuración
make gdb        # conecta GDB
