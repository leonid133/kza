
Silicon Laboratories C8051Fxxx uVISION DEBUG DRIVER v3.20 Release Notes

*Note: This debug driver is compatible with uVision2, uVision3 and uVision4.

KNOWN ISSUES AND LIMITATIONS
----------------------------
	
	1.) F00X, F01X, F02X, F04X, F06X, F12X, F13x, F2XX, F30X, F31X, 
            F32X, F33x, F34x, F36X, F35X, F41x, F50x, F52xA, F53xA, F54x,
            F55x, F56x, F57x, F58x, F59x, F70x, F71x, F80x, F81x, F82x,
            F83x, F93x, F92x, T60x, T61x and T63x devices are supported by this version. 

	2.) The call stack record facility of the on-chip debug
              logic is not supported.

	3.) Conditional data breakpoints (watchpoints) using the 
	    on-chip debug logic is not supported. (Note, this does 
	    not affect the ability to "watch" variables using the 
	    "watch window" facility of uVision.)

	4.) The double separator line in the "Peripherals" menu 
	    is a known issue.  This will be corrected by the next 
	    version of TP51.DLL by Keil Software. 

	5.) When setting breakpoints in uVision keep in mind 
              that although uVision will allow the unlimited setting of
              breakpoints Silicon Laboratories C8051Fxxx devices are 
              limited to 4 hardware breakpoints. Breakpoints are also  
              used to perform step over, run to main, and  run to   
              cursor operations. When more than 4 breakpoints are 
              attempted, an error message will be displayed when the 
              debugger tries to run, and until the excess breakpoints 
              are removed, the error will repeatedly be displayed 
              because uVision does not have a limit and will 
              continue to try to set more than 4 breakpoints. 

	6.) Corresponding with uVision Debug Driver release 1.9 is a 
	    change in installation strategy. This release will now be 
	    compatible with the release downloaded from www.silabs.com. 
	    Due to this change, release 1.9 will not recognize previous 
	    installations. The new installation will overwrite any 
	    existing files if installed to the same location as a 
	    previous installion. Attempting to uninstall a previous 
	    installation will affect the new installation if located in 
	    the same directory. 


HARDWARE SETUP
--------------
	
	EC2 Serial Adapter: Connect the EC2 Serial Adapter to the PC's 
	COM port using the RS232 serial cable and then to the target 
	device using the 10-pin ribbon cable.

	USB Debug Adapter: Connect the USB Debug Adapter to the PC's 
	USB port using the USB cable and then to the target device 
	using the attached 10-pin debug cable.


Release Dates
-------------


	uVision Debug Driver Version 3.20 - October 28, 2009
	uVision Debug Driver Version 3.14 - August 19, 2009
	uVision Debug Driver Version 3.10 - June 3, 2009
	uVision Debug Driver Version 3.00 - April 27, 2009
	uVision Debug Driver Version 2.90 - March 2, 2009
	uVision Debug Driver Version 2.82 - October 8, 2008
	uVision Debug Driver Version 2.81 - September 8, 2008
	uVision Debug Driver Version 2.80 - July 28, 2008
	uVision Debug Driver Version 2.73 - July 1, 2008
	uVision Debug Driver Version 2.72 - May 29, 2008
	uVision Debug Driver Version 2.71 - May 16, 2008
	uVision Debug Driver Version 2.70 - April 15, 2008
	uVision Debug Driver Version 2.60 - February 26, 2008
	uVision Debug Driver Version 2.51 - December 3, 2007	
	uVision Debug Driver Version 2.41 - October 31, 2007	
	uVision Debug Driver Version 2.40 - September 17, 2007
	uVision Debug Driver Version 2.31 - March 5, 2007
	uVision Debug Driver Version 2.30 - February 12, 2007
	uVision Debug Driver Version 2.21 - October 14, 2006


USING uVISION WITH SILICON LABORATORIES C8051Fxxx DEVICES
-------------------------------------------------------

	1.) Launch uVision2 (uv2.exe) or uVision3 (uv3.exe) or uVision4 (uv4.exe).

	2.) Create a project.   Use "Select Device for Target" from 
	    the "Project" menu to select a Silicon Laboratories C8051Fxxx 
	    device from the Device Database. (Refer to your Keil "Getting 
              Started with uVision2" manual for more information on creating 
	    and configuring projects.)

	3.) Open the "Options for Target" dialog from the "Project" menu
	    and select the "Debug" tab.

	4.) Select the "Silicon Laboratories C8051Fxxx uVision Driver" 
              in the drop-down list.  Click the "Use" radio button next to 
              "Silicon Laboratories C8051Fxxx uVision Driver".

	5.) Click the "Load Application at Startup" check box.  This 
	    must be selected to enable the download of your program 
	    to on-chip Flash memory.  Selecting "Go till main()" is 
	    optional. However, if "Go till Main" is not selected, 
              and a single step in the 'C' source is used to start 
              program execution, the first single step will appear 
              to be very slow. This is due to the fact that the 
              startup code is being executed, which actually consists 
              of approximately 300 assembly language instructions being 
              single stepped through.
	
	6.) Click on "Settings" and select the adapter you are using 
	    to connect to your device.

	7.) Close the "Options for Target" dialog and build the 
	    target program.

	8.) Select "Start/Stop Debug Session" from the "Debug" menu
	    to download your program to on-chip Flash program memory
	    and a begin a debug session.


SILICON LABORATORIES C8051Fxxx uVISION DRIVER REVISION HISTORY
--------------------------------------------------------------

Version 3.20.00
        Added support for C8051F34C/D devices.
        Added support for C8051F800/1/2/3/4/5/6/7/8/9 devices.
        Added support for C8051F810/1/2/3/4/5/6/7/8/9 devices.
        Added support for C8051F820/1/2/3/4/5/6/7/8/9 devices.
        Added support for C8051F830/1/2/3/4/5 devices.
        Added support for C8051F901/2 devices.
        Added support for C8051F911/2 devices.


	Note: This release includes a database which can be used with uVision 4.
	Due to a change in uVision some of the device menus which were previously
	located in the Peripherals menu are now located in the Debug menu under 
	Debug Settings.


Version 3.14.00
	Fixed breakpoint operation on C8051F580-91 devices.

Version 3.10.00
        Added support for C8051F540/1/2/3/4/5/6/7 devices.
        Added support for C8051F550/1/2/3/4/5/6/7 devices.
        Added support for C8051F560/1/2/3/4/5/6/7/8/9 devices.
        Added support for C8051F570/1/2/3/4/5 devices.

Version 3.00.00
        Added support for C8051T606 device.

Version 2.90.00
	Added support for C8051F508/9 devices.
	Added support for C8051F510/1 devices.
	Added support for C8051F580/1/2/3/4/5/6/7/8/9 devices.
	Added support for C8051F590/1 devices.
	Added support for C8051F700/1/2/3/4/5/6/7/8/9 devices.
	Added support for C8051F710/1/2/3/4/5 devices.


Version 2.82
	Support added for C8051F34A/B devices.

	Corrections
	-----------
	Changed title to "Silicon Labs C8051Fxxx Driver" to fit in Keil uVision list boxes.	
	
	Fixed memory size configuration in T61x devices. T61x devices can now be programmed without
	a CRC error message.



Version 2.81
	Firmware for the EC2 Serial Adapter has been updated to Version 22.
	Firmware for the USB Debug Adapter has been updated to Version 20.
	Firmware for the Toolstick Adapter has been updated to Version 5.


Version 2.80
	Support added for C8051F500/1/2/3/4/5/6/7 devices.

	Firmware for the EC2 Serial Adapter has been updated to Version 21.
	Firmware for the USB Debug Adapter has been updated to Version 19.
	Firmware for the Toolstick Adapter has been updated to Version 4.

	When the Flash menu Download is used, a checksum is displayed in the output window.

Version 2.73
	Support added for C8051F348/9 devices.

Version 2.72
	Device XRAM access problem fixed.

Version 2.71
	JTAG Chaining problem has been fixed.

Version 2.70
	Support added for C8051F520A/1A/3A/4A/6A/7A devices.
	Support added for C8051F530A/1A/3A/4A/6A/7A devices.

	Problem editing SFR values from peripheral windows fixed.

Version 2.60
	Support added for C8051T630/1/2/3/4/5 devices.
	Support added for C8051F930/1 devices.
	Support added for C8051F920/1 devices.

	Problems using the load button and flash menu download command to program 
          flash have been fixed.

Version 2.51
	A problem that caused Register and SFR values to not be correctly updated 
	during a debug session has been fixed.

Version 2.50
	Support added for C8051T610/1/2/3/4/5/6/7 devices.
	An error that occurred when starting a debug session on 41x, 25x, 52x/53x,
	and 36x devices has been fixed.

Version 2.41
	A problem that caused an error when the flash menu download button was used and 
	erase was also performed has been fixed.

Version 2.40
	Support added for C8051F336/7/8/9 devices.

Version 2.31
	Support added for C8051T600/1/2/3/4/5 devices.

Version 2.30
	Support added for C8051F360/1/2/3/4/5/6/7/8/9 devices.

Version 2.21
	Support added for C8051F520/1/3/4/6/7 devices.
	Support added for C8051F530/1/3/4/6/7 devices.


Version 2.2
	A ToolStick device can be used as a USB Debug Adapter.	
	Firmware for the EC2 Serial Adapter has been updated to Version 20.
	Firmware for the USB Debug Adapter has been updated to version 12.

Version 2.1
	Issue when viewing XRAM with C8051F34x devices fixed.


Version 2.0
	Support added for C8051F410/1/2/3 devices. 


Version 1.9
	Support added for C8051F340/1/2/3/4/5/6/7 devices. 
	Support added for C8051F326/7 devices.	

	Firmware for the EC2 Serial Adapter has been updated to Version 19.
	Firmware for the USB Debug Adapter has been updated to version 9.

	If a flash erase is cancelled, the driver will not report that flash 
	has been erased.

	A problem which prevented connection to F332/3/4/5 devices has been fixed.
	A problem which prevented connection to a COMM port above 4 has been corrected.

	Installer created to support uVision3.

Version 1.81
	A problem which prevented V1.8 from connecting to C8051F064 evaluation boards
	has been corrected.


Version 1.8
	Support added for Si825x devices. 
	Support added for C8051F316/7 devices.

          The extended memory interface for C8051F13x devices was fixed to allow 
	external memory to be displayed up to 64k.

	Firmware for the EC2 Serial Adapter has been updated to Version 18.
	Firmware for the USB Debug Adapter has been updated to version 7.
	The new firmware versions include speed improvements for JTAG devices.

	The USBHID.DLL has been updated to version 1.1.0.0. Version 1.8 of the 
	DLL requires this version of the USBHID.DLL and will not work with older
	versions.

Version 1.71
	License Agreement updated.

Version 1.7
	The settings dialog has been enhanced to allow the selection of COM ports above 4.
 
	Support added for the USB debug adapter.

	A problem which prevented uVision to correctly debug banked projects on 130/1 devices
	was corrected.

	Firmware for the EC2 Serial Adapter has been updated to Version 16.

Version 1.6
	A problem which prevented the DLL from connecting to F304 and F305 devices was corrected.

	Support updated for the C8051F33x device family.

	A RAM corruption problem which occurred when watch window variables were edited has been fixed.

	If a breakpoint is set while the MCU is running, the breakpoint will not take effect until the
	target has been stopped and restarted.

Version 1.5
	Support has been added for F044/5/6/7 devices.
	Support has been added for F312/3/4/5 devices.

Version 1.4
	Support has been added for F130/1/2/3 devices.

Version 1.39
	Support has been added for F064/5/6/7 devices.

Version 1.38
	SFR read accesses for paged devices were modified to return 
	values based on the user's sfrpage sfr value.

Version 1.37
          DLL name has been changed from CygC8051F.dll to SiC8051F.dll.

	Support has been added for F35X devices.

	If flash programming is cancelled due to write locked flash,
	the debugging session is aborted. To debug a device which has
	write locked flash, the "Load application at startup" option 
	must not be selected.

Version 1.36
	An error that resulted when using the F10 key for single-step
	operations while 'hovering' the mouse over a code variable
	has been fixed.

Version 1.35
	A problem with source level debugging above address 0x8000 
	on devices with more than 64k, but which were not enabling 
	banking has been corrected.

Version 1.34
	A flash erase operation can now be performed on a device
	which contains locked pages, as long as page 0 (address 0)
          is not locked.

	Breakpoint operation has been improved. If a debug session is
	in progress, breakpoints will be set and cleared in 'real' time.

Version 1.33
	An error which caused incorrect updates of the Port 1
	configuration register on the Port 1 peripheral dialog
	for F00x and F01x devices has been fixed.

Version 1.32
	Support has been added for F06X devices.

	An error preventing the Port 1 peripheral dialog from 
	displaying correctly on F00x and F01x devices has been fixed.

Version 1.31
	Support has been added for F04X devices.

          The trace record information of the on-chip debug logic 
          for devices in the 04X & 12X families is now supported.


Version 1.30
	A driver installation program has been implemented. 
	The following files are included in the installation:
	CygC8051F.dll - Copied into the C51\Bin directory of an 
		existing Keil installation. The tools.ini file
		will be edited so that the C8051Fxxx Driver
		can be identified by uVision2.
	uv2.cdb - This is the standard database definition file 
		from Keil and is copied into the UV2 directory 
		only if it is a newer version than what is 
		already present in the existing Keil installation.
	tcyg.dll,dcyg.dll - DLL's provided by Keil which define the
		dialog peripherals used for C8051Fxxx devices. They are
		copied into the C51\Bin directory only is they are
		newer than the existing versions.
	Readme_for_SiC8051FDLL.txt - The release notes for Si8051F.dll. 
		This file is copied into the C51\Hlp directory of the
		Keil installation and contains information about using
		uVision with Silicon Laboratories C8051Fxxx devices and 
		the revision history for the DLL. 

	A problem that caused the DLL to hang when a breakpoint was
	set which would never be reached has been fixed.

Version 1.29
	Support has been added for F32X & F33X devices. The C8051F320/21/
          22/23 and C8051F330/31 devices have been added to the database.

          The DLL now supports the new uV2 flash interface. (Version 2.34
	of uV2 is required.)

	A problem which caused peripheral dialog values to display zeros 
	when first displayed has been fixed.

	The reset button is no longer enabled when the target is running.

	A program not responding error that occurred when uV2 was not the 
	active Windows application during flash downloads has been fixed.

	If the target board is disconnected while running the DLL will be
	shut down.

Version 1.28
	Support has been added for F31X devices. The C8051F310 and 
	C8051F311 devices have been added to the database.

          The cache data view for 12x devices has been implemented.


Version 1.27
	Support has been added for F12X devices.

	The following modifications were made to the database:
	The multi-device selections were changed to individual
	descriptions. The following devices were added to 
	the database: C8051F018 & C8051F019 
	C8051F120, C8051F121, C8051F122, C8051F123 
	C8051F124, C8051F125, C8051F126, C8051F127.  

	Scratch pad memory on the F02X & F12X devices is accessed 
	in the memory window using the 's' designator (s:0x0000). 
	uVision2 V2.20 or greater is required to view the scratch-pad
	memory space.

Version 1.26
	A problem with external memory accesses that could cause memory 
	window contents to wrap has been fixed.

Version 1.25
	Database & DLL updated to match new Keil release chip definitions.

	Support has been added for F018 & F019 devices.

	Settings dialog now includes a "Cache Code Memory" option.
	The default is enabled which provides a noticeable speed
 	improvement. However, the cache feature should not be used 
	if code is modifying flash memory contents, because updates 
	may not be reflected.

	The serial adapter firmware embedded in the DLL has been 
	updated to Version 0x0F.

Version 1.24
	The serial adapter firmware embedded in the DLL has been 
	updated to Version 0x0E.

Version 1.23
	The serial adapter firmware embedded in the DLL has been 
	updated to Version 0x0D.

Version 1.22
	Single stepping using a F30X device now correctly updates
	all values on the 'Regs' tab of the Project window. 

Version 1.21
	Support for the scratch pad memory on the F02X devices has
	been implemented. The scratch pad memory is accessed in the
	memory window using the 'c' designator (c:0x10000), with an 
	address in the range of 0x10000 to 0x10080. Memory window
	content modification problem has been fixed. Incorrect PSW 
	value in the Regs tab of the Project Window has been fixed. 

Version 1.2
	Support has been added for F02X & F30X devices.
          Flash programming has been enhanced. The download algorithm
	has been enhanced to improve speed. When flash is being 
          programmed, a status message and progress indication has been
          added. DPTR value is now correctly displayed. Modifications
          to the code space are now updated during step operations.
	A progress dialog was added when a firmware upgrade is 
	performed.

Version 1.1
	External memory modifications typed into a memory window are 
	now persistant. The direct memory accesses in a memory window 
	(d:0x00 - d:0xFF) now are correctly interpreted as follows: 
	0x00 - 0x7f (RAM), 0x80 - 0xFF (SFR).

Version 1.0
	Initial release.
	



