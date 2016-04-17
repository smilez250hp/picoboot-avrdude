This is a mirror/fork of the <a href='http://savannah.nongnu.org/svn/?group=avrdude'>avrdude repository</a> with <a href='https://code.google.com/p/picoboot/'>picoboot bootloader</a> support added.  A working configure script is included so bootstrap does not have to be run to generate the configure script.
It is based on avrdude RELEASE\_6\_1 from savannah.nongnu.org.  Windows and Linux binaries are available here: http://162.248.164.251/files/

Example usage under Windows:
avrdude -c picoboot -p t88 -P com11 -U flash:w:blink\_led-t88.hex