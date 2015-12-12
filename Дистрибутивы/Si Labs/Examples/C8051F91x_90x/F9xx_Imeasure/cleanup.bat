REM Can give operand to point to the appropriate directory
cd %~f1

REM This will remove things Keil generates
del *.#1
del *.#2
del *.#3
del *.lst
del *.m51
del cyglink.txt
del *.obj
del tmp.out
del *.src

del *.
del *.5MHz
del *.hex


REM This will remove things that SDCC generates
del *.adb
del *.cdb
del *.rel
del *.sym
del *.asm
del *.map
del *.mem
del *.rst
exit