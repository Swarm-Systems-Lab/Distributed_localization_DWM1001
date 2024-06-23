### Current issues

* **OpenOCD debugging Error: No symbols for ChibiOS**
  
  At connection with GDB session, symbols needed on [OpenOCD documentation](https://openocd.org/doc/html/GDB-and-OpenOCD.html). ```rlist``` included in ```ch```. [Relevant code](https://openocd.org/doc-release/doxygen/ChibiOS_8c_source.html)

* **OpenOCD debugging Warn : ChibiOS/RT memory signature claims to be bigger than expected.**
  
  At connection with GDB session, memory signature has mismatch between reported size inside own struct and real size. [Relevant code](https://openocd.org/doc-release/doxygen/ChibiOS_8c_source.html)

* **OpenOCD debugging Error: ChibiOS registry integrity check failed, double linked list violation**
  
  At connection with GDB session, maybe due to bad initialization or memory corruption. [ChibiOS Forum](https://forum.chibios.org/viewtopic.php?t=2422)
