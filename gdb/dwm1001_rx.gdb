file "./build/dis_loc_dwm1001_rx.elf"

set architecture auto
set remote hardware-breakpoint-limit 6
set remote hardware-watchpoint-limit 4

target extended-remote :3333

load
tbreak main

continue

