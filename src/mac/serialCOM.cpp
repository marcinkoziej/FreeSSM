/*
 * serialCOM.cpp - Serial port configuration and communication
 *
 * Copyright (C) 2008-2023 Comer352L
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "serialCOM.h"

#include <algorithm>


serialCOM::serialCOM()
{
	fd = -1;
	portisopen = false;
	breakset = false;
	currentportname = "";
	memset(&oldtio, 0, sizeof(oldtio));
	settingssaved = false;
	currentbaudrate = 0;
}


serialCOM::~serialCOM()
{
	if (portisopen) ClosePort();
}


std::vector<std::string> serialCOM::GetAvailablePorts()
{
	std::vector<std::string> portlist(0);
	int testfd = -1;				// file descriptor for tested device files
	char ffn[256] = "";				// full filename incl. path
	DIR *dp = NULL;
	struct dirent *fp = NULL;
	dp = opendir("/dev");
	if (dp != NULL)
	{
		do {
			fp = readdir(dp);
			if (fp != NULL)
			{
				// macOS exposes serial devices as /dev/cu.* (call-out, no carrier-detect wait)
				// and /dev/tty.* (dial-in, waits for DCD). Use cu.* for outgoing connections.
				if (!strncmp(fp->d_name, "cu.", 3))
				{
					strcpy(ffn, "/dev/");
					strcat(ffn, fp->d_name);
					testfd = open(ffn, O_RDWR | O_NOCTTY | O_NDELAY);
					if (testfd != -1)
					{
						if (isatty(testfd))
							portlist.push_back(ffn);
						close(testfd);
					}
				}
			}
		} while (fp != NULL);
		closedir(dp);
	}
#ifdef __SERIALCOM_DEBUG__
	else
		std::cout << "serialCOM::GetAvailablePorts():   opendir(\"/dev\") failed with error " << errno << " " << strerror(errno) << "\n";
#endif
	std::sort(portlist.begin(), portlist.end());
	return portlist;
}


bool serialCOM::IsOpen()
{
	if (portisopen && (fd >= 0))
		return true;
	else
		return false;
}


std::string serialCOM::GetPortname()
{
	return currentportname;
}


bool serialCOM::GetPortSettings(double *baudrate, unsigned short *databits, char *parity, float *stopbits)
{
	/* NOTE: NULL-pointer arguments allowed ! */
	bool settingsvalid = true;
	struct termios currenttio;
	memset(&currenttio, 0, sizeof(currenttio));
	unsigned int cleanedbitmask = 0;
	if (!portisopen) return false;
	// Query current settings:
	if (tcgetattr(fd, &currenttio) == -1)
	{
#ifdef __SERIALCOM_DEBUG__
		std::cout << "serialCOM::GetPortSettings():   tcgetattr(...) failed with error " << errno << " " << strerror(errno)<< "\n";
#endif
		return false;
	}
	// BAUD RATE:
	if (baudrate)
		*baudrate = currentbaudrate;
	// DATABITS:
	if (databits)
	{
		cleanedbitmask = (currenttio.c_cflag & ~CSIZE);
		if (currenttio.c_cflag == (cleanedbitmask | CS8))
		{
			*databits = 8;
		}
		else if (currenttio.c_cflag == (cleanedbitmask | CS7))
		{
			*databits = 7;
		}
		else if (currenttio.c_cflag == (cleanedbitmask | CS6))
		{
			*databits = 6;
		}
		else if (currenttio.c_cflag == (cleanedbitmask | CS5))
		{
			*databits = 5;
		}
		else
		{
			settingsvalid = false;
#ifdef __SERIALCOM_DEBUG__
			std::cout << "serialCOM::GetPortSettings():   error: unknown number of databits\n";
#endif
		}
	}
	if (parity)
	{
		if (currenttio.c_cflag != (currenttio.c_cflag | PARENB)) // if parity-flag is not set
			*parity='N';
		else    	// if parity-flag is set
		{
			if (currenttio.c_cflag == (currenttio.c_cflag | PARODD)) // if Odd-(Mark-) parity-flag is set
			{
#ifdef CMSPAR
				if (currenttio.c_cflag == (currenttio.c_cflag | CMSPAR)) // if Space/Mark-parity-flag is set
					*parity='M';
				else
#endif
					*parity='O';
			}
			else	// if Odd-(Mark-) parity-flag is not set
			{
#ifdef CMSPAR
				if (currenttio.c_cflag == (currenttio.c_cflag | CMSPAR)) // if Space/Mark-parity-flag is set
					*parity='S';
				else

#endif
					*parity='E';
			}
		}
	}
	// STOPBITS:
	if (stopbits)
	{
		if (currenttio.c_cflag != (currenttio.c_cflag | CSTOPB))
			*stopbits = 1;
		else
		{
			if (*databits == 5)
			{
				*stopbits = 1.5;
			}
			else if (*databits > 5)
			{
				*stopbits = 2;
			}
			else
			{
				settingsvalid = false;
				*stopbits = 0;
			}
		}
	}
	// RETURN SUCCESS:
	return settingsvalid;
}


bool serialCOM::SetPortSettings(double baudrate, unsigned short databits, char parity, float stopbits)
{
	int cTCSETATTR = -1;
	int cIOCTL = -1;
	bool isStdBaud = true;
	bool settingsvalid = true;
	speed_t newbaudrate = 0;
	struct termios newtio;
	memset(&newtio, 0, sizeof(newtio));
	if (!portisopen) return false;
	// SET CONTROL OPTIONS:
	newtio.c_cflag = (CREAD | CLOCAL);
	// BAUD RATE:
	// Set new baud rate:
	if (baudrate <= 0)
	{
		settingsvalid = false;
#ifdef __SERIALCOM_DEBUG__
		if (baudrate == 0)
			std::cout << "serialCOM::SetPortSettings:   error: illegal baud rate - 0 baud not possible\n";
		else
			std::cout << "serialCOM::SetPortSettings:   error: illegal baud rate - baud rate must be > 0\n";
#endif
	}
	else
	{
		isStdBaud = GetStdBaudRateDCBConst(baudrate, &newbaudrate);
		if (!isStdBaud)
		{
#ifdef __SERIALCOM_DEBUG__
			std::cout << "serialCOM::SetPortSettings:   encoding non-standard baud rate with IOSSIOSPEED\n";
#endif
			newbaudrate = B9600;
		}
	}
	// DATABITS:
	switch (databits)
	{
		case 8:
			newtio.c_cflag |= CS8;
			break;
		case 7:
			newtio.c_cflag |= CS7;
			break;
		case 6:
			newtio.c_cflag |= CS6;
			break;
		case 5:
			newtio.c_cflag |= CS5;
			break;
		default:
			settingsvalid = false;
#ifdef __SERIALCOM_DEBUG__
			std::cout << "serialCOM::SetPortSettings():  error: illegal number of databits\n";
#endif
			break;
	}
	// PARITY:
	if (parity == 'N')
	{
		newtio.c_cflag &= ~PARENB;	// deactivate parity (not really necessary, because c_cflag is clean)
	}
	else if (parity == 'E')
	{
		newtio.c_cflag |= PARENB;	// activate parity
#ifdef CMSPAR
		newtio.c_cflag &= ~CMSPAR;	// activate mark/space mode (not really necessary, because c_cflag is clean)
#endif
		newtio.c_cflag &= ~PARODD;	// deactivate odd parity (not really necessary, because c_cflag is clean)
	}
	else if (parity == 'O')
	{
		newtio.c_cflag |= PARENB;	// activate parity
#ifdef CMSPAR
		newtio.c_cflag &= ~CMSPAR;	// deactivate mark/space mode (not really necessary, because c_cflag is clean)
#endif
		newtio.c_cflag |= PARODD;	// activate odd parity
	}
#ifdef CMSPAR
	else if (parity == 'S')
	{
		newtio.c_cflag |= PARENB;	// activate parity
		newtio.c_cflag |= CMSPAR;	// activate mark/space mode
		newtio.c_cflag &= ~PARODD;	// deactivate mark parity (not really necessary, because c_cflag is clean)
	}
	else if (parity == 'M')
	{
		newtio.c_cflag |= PARENB;	// activate parity
		newtio.c_cflag |= CMSPAR;	// activate mark/space mode
		newtio.c_cflag |= PARODD;	// activate mark parity
	}
#endif
	else
	{
		settingsvalid = false;
#ifdef __SERIALCOM_DEBUG__
		std::cout << "serialCOM::SetPortSettings():  error: illegal parity option\n";
#endif
	}
	// STOPBITS:
	if (stopbits == 1)
	{
		newtio.c_cflag &= ~CSTOPB;	//  (not really necessary, because c_cflag is clean)
	}
	else if (stopbits == 1.5)
	{
		if (databits == 5)
			newtio.c_cflag |= CSTOPB;
		else
		{
			settingsvalid = false;
#ifdef __SERIALCOM_DEBUG__
			std::cout << "serialCOM::SetPortSettings():   error: invalid value for parameter 'stopbits': 1.5 stopbits only allowed in combination with 5 databits\n";
#endif
		}
	}
	else if (stopbits == 2)
	{
		if (databits != 5)
			newtio.c_cflag |= CSTOPB;
		else
		{
			settingsvalid = false;
#ifdef __SERIALCOM_DEBUG__
			std::cout << "serialCOM::SetPortSettings():   error: invalid value for parameter 'stopbits': 2 stopbits not allowed in combination with 5 databits\n";
#endif
		}
	}
	else
	{
		settingsvalid = false;
#ifdef __SERIALCOM_DEBUG__
		std::cout << "serialCOM::SetPortSettings():   error: invalid value for parameter 'stopbits'" << '\n';
#endif
	}
   /*	CBAUD	Bit mask for baud rate
	B0	0 baud (drop DTR)
	B50	50 baud
	B75	75 baud
	B110	110 baud
	B134	134.5 baud
	B150	150 baud
	B200	200 baud
	B300	300 baud
	B600	600 baud
	B1200	1200 baud
	B1800	1800 baud
	B2400	2400 baud
	B4800	4800 baud
	B9600	9600 baud
	B19200	19200 baud
	B38400	38400 baud
	B57600	57,600 baud
	B76800	76,800 baud
	B115200	115,200 baud
	EXTA	External rate clock
	EXTB	External rate clock
	CSIZE	Bit mask for data bits
	CS5	5 data bits
	CS6	6 data bits
	CS7	7 data bits
	CS8	8 data bits
	CSTOPB	2 stop bits (1 otherwise)
	CREAD	Enable receiver
	PARENB	Enable parity bit
	PARODD	Use odd parity instead of even
	CMSPAR	Use space/mark parity instead of even/odd		// not POSIX-defined ! (undocumented !)
	HUPCL	Hangup (drop DTR) on last close
	CLOCAL	Local line - do not change "owner" of port
	LOBLK	Block job control output
	CRTSCTS	Enable hardware flow control (not supported on all platforms)
	CNEW_RTSCTS		// not available under Linux				*/
// LINE OPTIONS (LOCAL OPTIONS) - controls how input characters are managed:
	newtio.c_lflag = NOFLSH;
   /*	ISIG	Enable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals
	ICANON	Enable canonical input (else raw)
	XCASE	Map uppercase \lowercase (obsolete)
	ECHO	Enable echoing of input characters
	ECHOE	Echo erase character as BS-SP-BS
	ECHOK	Echo NL after kill character
	ECHONL	Echo NL
	NOFLSH	Disable flushing of input buffers after interrupt or quit characters
	IEXTEN	Enable extended functions (this flag, as well as ICANON must be enabled for the
		 special characters EOL2, LNEXT, REPRINT, WERASE to be interpreted, and for the IUCLC
		 flag to be effective)
	ECHOCTL	Echo control characters as ^char and delete as ~?
	ECHOPRT	Echo erased character as character erased
	ECHOKE	BS-SP-BS entire line on line kill
	FLUSHO	Output being flushed
	PENDIN	Retype pending input at next read or input char
	TOSTOP	Send SIGTTOU for background output				*/
// INPUT OPTIONS:
	newtio.c_iflag = IGNPAR | IGNBRK;
   /*	INPCK	Enable parity check
	IGNPAR	Ignore parity errors
	PARMRK	Mark parity errors
	ISTRIP	Strip parity bits
	IXON	Enable software flow control (outgoing)
	IXOFF	Enable software flow control (incoming)
	IXANY	Allow any character to start flow again
	IGNBRK	Ignore break condition
	BRKINT	Send a SIGINT when a break condition is detected
	INLCR	Map NL to CR
	IGNCR	Ignore CR
	ICRNL	Map CR to NL
	IUCLC	Map uppercase to lowercase
	IMAXBEL	Echo BEL on input line too long
	IUTF8				*/
// OUTPUT OPTIONS:
	newtio.c_oflag &= ~OPOST;  // => raw output - all other option bits will be ignored
   /*	OPOST	Postprocess output (not set = raw output)
	OLCUC	Map lowercase to uppercase (not POSIX ???)
	ONLCR	Map NL to CR-NL
	OCRNL	Map CR to NL
	NOCR	No CR output at column 0
	ONOCR
	ONLRET	NL performs CR function
	OFILL	Use fill characters for delay
	OFDEL	Fill character is DEL
	NLDLY	Mask for delay time needed between lines
	NL0	No delay for NLs
	NL1	Delay further output after newline for 100 milliseconds
	CRDLY	Mask for delay time needed to return carriage to left column
	CR0	No delay for CRs
	CR1	Delay after CRs depending on current column position
	CR2	Delay 100 milliseconds after sending CRs
	CR3	Delay 150 milliseconds after sending CRs
	TABDLY	Mask for delay time needed after TABs
	TAB0	No delay for TABs
	TAB1	Delay after TABs depending on current column position
	TAB2	Delay 100 milliseconds after sending TABs
	TAB3	Expand TAB characters to spaces
	BSDLY	Mask for delay time needed after BSs
	BS0	No delay for BSs
	BS1	Delay 50 milliseconds after sending BSs
	VTDLY	Mask for delay time needed after VTs
	VT0	No delay for VTs
	VT1	Delay 2 seconds after sending VTs
	FFDLY	Mask for delay time needed after FFs
	FF0	No delay for FFs
	FF1	Delay 2 seconds after sending FFs
// CONTROL CHARACTERS AND TIMOUT OPTIONS:		*/
	// Timeout settings (for input):
	newtio.c_cc[VMIN] = 0;
	newtio.c_cc[VTIME] = 0;
   /*	VINTR		Interrupt				CTRL-C
	VQUIT		Quit					CTRL-Z
	VERASE		Erase					Backspace (BS)
	VKILL		Kill-line				CTRL-U
	VEOF		End-of-file				CTRL-D
	VEOL		End-of-line				Carriage return (CR)
	VEOL2		Second end-of-line			Line feed (LF)
	VMIN		Minimum number of characters to read	-
	VSTART		Start flow				CTRL-Q (XON)
	VSTOP		Stop flow				CTRL-S (XOFF)
	VTIME		Time to wait for data (1/10 seconds)	-
	VSWTC							\0
	VSUSP							CTRL-z
	VREPRINT						CTRL-r
	VDISCARD						CTRL-u
	VWERASE							CTRL-w
	VLNEXT							CTRL-v
	...									*/
	if (settingsvalid == true)	// apply new settings only if they are all valid !
	{
		if (cfsetispeed(&newtio, newbaudrate) != 0)
		{
#ifdef __SERIALCOM_DEBUG__
			std::cout << "serialCOM::SetPortSettings():   cfsetispeed(...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
			settingsvalid = false;
		}
		if (settingsvalid && (cfsetospeed(&newtio, newbaudrate) != 0))
		{
#ifdef __SERIALCOM_DEBUG__
			std::cout << "serialCOM::SetPortSettings():   cfsetospeed(...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
			settingsvalid = false;
		}
		if (settingsvalid)
		{
			cTCSETATTR = tcsetattr(fd, TCSANOW, &newtio);
#ifdef __SERIALCOM_DEBUG__
			if (cTCSETATTR == -1)
				std::cout << "serialCOM::SetPortSettings():   tcsetattr(...) failed with error " << errno << " " << strerror(errno)<< "\n";
#endif
			if ((cTCSETATTR == 0) && (!isStdBaud))
			{
				speed_t customspeed = static_cast<speed_t>(round(baudrate));
				cIOCTL = ioctl(fd, IOSSIOSPEED, &customspeed);
#ifdef __SERIALCOM_DEBUG__
				if (cIOCTL == -1)
					std::cout << "serialCOM::SetPortSettings():   ioctl(..., IOSSIOSPEED, ...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
			}
			if ((cTCSETATTR == 0) && (isStdBaud || (cIOCTL != -1)))
				currentbaudrate = baudrate;
		}
	}
	// SUCCESS CONTROL (RETURN VALUE):
	return ((cTCSETATTR == 0) && (isStdBaud || (cIOCTL != -1)));
}


bool serialCOM::OpenPort(std::string portname)
{
	int confirm = -1;	// -1=error
	if (portisopen) return false;	// if port is already open => cancel and return "false"
	memset(&oldtio, 0, sizeof(oldtio));
	// OPEN PORT:
	fd = open(portname.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	/* 		O_RDWR	 = read/write access;
	*		O_NOCTTY = ignore control characters
	*		O_NDELAY = ignore DCD-line		*/
	if (!(fd < 0))	// if port is now open
	{
		portisopen = true;
		currentportname = portname;
		confirm = ioctl(fd, TIOCEXCL, NULL);	// LOCK DEVICE
#ifdef __SERIALCOM_DEBUG__
		if (confirm == -1)
			std::cout << "serialCOM::OpenPort():   ioctl(..., TIOCEXCL, ...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
		confirm = fcntl(fd, F_SETFL, FNDELAY);	// function "read" shall return "0" if no data available
		if (confirm == -1)
		{
#ifdef __SERIALCOM_DEBUG__
			std::cout << "serialCOM::OpenPort():   fcntl(...) failed with error " << errno << "\n";
#endif
			confirm = ClosePort();
#ifdef __SERIALCOM_DEBUG__
			if (!confirm)
				std::cout << "serialCOM::OpenPort():   port couldn't be closed after error during opening process\n";
#endif
			return false;
		}
		// SAVE SETTINGS:
		confirm = tcgetattr(fd, &oldtio);
		if (confirm == -1)
		{
#ifdef __SERIALCOM_DEBUG__
			std::cout << "serialCOM::OpenPort():   tcgetattr(...) failed with error " << errno << "\n";
#endif
			settingssaved = false;
		}
		else
			settingssaved = true;
		breakset = false;
		// CLEAR HARDWARE BUFFERS:
		confirm = tcflush(fd, TCIOFLUSH);
		if (confirm == -1)
		{
#ifdef __SERIALCOM_DEBUG__
			std::cout << "serialCOM::OpenPort():   tcflush(..., TCIOFLUSH) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
			confirm = ClosePort();
#ifdef __SERIALCOM_DEBUG__
			if (!confirm)
				std::cout << "serialCOM::OpenPort():   port couldn't be closed after error during opening process\n";
#endif
			return false;
		}
		// CONFIGURE COMMUNICATION, SET STANDARD PORT-SETTINGS
		if (!SetPortSettings(9600, 8, 'N', 1 ))
		{
#ifdef __SERIALCOM_DEBUG__
			std::cout << "serialCOM::OpenPort():   Couldn't set standard port settings with SetPortSettings() !\n";
#endif
			confirm = ClosePort();
#ifdef __SERIALCOM_DEBUG__
			if (!confirm)
				std::cout << "serialCOM::OpenPort():   port couldn't be closed after error during opening process\n";
#endif
			return false;
			/* NOTE: SetPortSettings not only changes the 4 communication parameters.
			         It configures additional parameters (like control characters, timeouts, ...) which are
			         are important to ensure proper communication behavior !
			 */
		}
		// SET CONTROL LINES (DTR+RTS) TO STANDARD VALUES:
		confirm = SetControlLines(true, true);
#ifdef __SERIALCOM_DEBUG__
		if (!confirm)
			std::cout << "serialCOM::OpenPort():   Warning: couldn't set RTS+DTS control lines to standard values\n";
#endif
		/* NOTE: Call SetControlLines AFTER SetPortSettings, because drivers can
		 * change DTS+RTS when new baud rate / databits / parity / stopbits,
		 * especially at the first time after opening the port !		*/
		return true;
	}
	else
	{
#ifdef __SERIALCOM_DEBUG__
		std::cout << "serialCOM::OpenPort():   open(...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
		return false;
	}
}


bool serialCOM::ClosePort()
{
	int confirm = -1;	// -1=error
	if (!portisopen) return false;
	// CLEAR BREAK:
	confirm = ioctl(fd, TIOCCBRK, 0);	// break OFF
	if (confirm != -1)
		breakset = false;
#ifdef __SERIALCOM_DEBUG__
	else
		std::cout << "serialCOM::ClosePort():   ioctl(..., TIOCCBRK, ...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
	// CLEAR HARDWARE BUFFERS:
	confirm = tcflush(fd, TCIOFLUSH);	// clear buffers (input and output)
#ifdef __SERIALCOM_DEBUG__
	if (confirm == -1)
		std::cout << "serialCOM::ClosePort():   tcflush(..., TCIOFLUSH) failed with error " << errno << "\n";
#endif
	// RESTORE PORT SETTINGS:
	if (settingssaved)
	{
		confirm = tcsetattr(fd, TCSANOW, &oldtio);	// restore the old port settings
#ifdef __SERIALCOM_DEBUG__
	if (confirm == -1)
		std::cout << "serialCOM::ClosePort():   tcsetattr(...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
	}
	confirm = ioctl(fd, TIOCNXCL, NULL);	// unlock device
#ifdef __SERIALCOM_DEBUG__
	if (confirm == -1)
		std::cout << "serialCOM::ClosePort():   ioctl(..., TIOCNXCL, ...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
	// CLOSE PORT:
	confirm = close(fd);
	// CLEAN UP, RETURN VALUE:
	if (confirm==0)
	{
		fd = -1;
		portisopen = false;
		breakset = false;
		currentportname = "";
		currentbaudrate = 0;
		return true;
	}
	else
	{
#ifdef __SERIALCOM_DEBUG__
		std::cout << "serialCOM::ClosePort():   close(...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
		return false;
	}
}


bool serialCOM::Write(std::vector<char> data)
{
	return Write(&data.at(0), data.size());
}


bool serialCOM::Write(char *data, unsigned int datalen)
{
	int confirm = -1;
	unsigned int nrofbyteswritten = 0;
	if (!portisopen) return false;
	if (breakset)
	{
		confirm = ioctl(fd, TIOCCBRK, 0);	// break OFF
		if (confirm != -1)
			breakset = false;
		else
		{
#ifdef __SERIALCOM_DEBUG__
			std::cout << "serialCOM::Write():   ioctl(..., TIOCCBRK, ...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
			return false;
		}
	}
	// SEND DATA:
	nrofbyteswritten = write(fd, data, datalen);
	// WAIT UNTIL ALL BYTES ARE TRANSMITTED (to the buffer !?):
	confirm = tcdrain(fd);
	// RETURN VALUE:
	if ((nrofbyteswritten == datalen) && (confirm == 0))
		return true;
	else
	{
#ifdef __SERIALCOM_DEBUG__
		if (nrofbyteswritten != datalen)
			std::cout << "serialCOM::Write():   write(...) failed with error " << errno << " " << strerror(errno) << "\n";
		if (confirm != 0)
			std::cout << "serialCOM::Write():   tcdrain(...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
		return false;
	}
}


bool serialCOM::Read(unsigned int minbytes, unsigned int maxbytes, unsigned int timeout, std::vector<char> *data)
{
	if (!portisopen || (minbytes > maxbytes) || (maxbytes > INT_MAX))
		return false;
	unsigned int rdatalen = 0;
	char *rdata = (char*) malloc(maxbytes);
	if (rdata == NULL) return false;
	bool ok = Read(minbytes, maxbytes, timeout, rdata, &rdatalen);
	if (ok)	data->assign(rdata, rdata+rdatalen);
	free(rdata);
	return ok;
}


bool serialCOM::Read(unsigned int minbytes, unsigned int maxbytes, unsigned int timeout, char *data, unsigned int *nrofbytesread)
{
	int ret;
	*nrofbytesread = 0;
	if (!portisopen || (minbytes > maxbytes) || (maxbytes > INT_MAX))
		return false;

	if (!minbytes)
	{
		ret = read(fd, data, maxbytes);
		if ((ret >= 0) && (ret <= static_cast<int>(maxbytes)))	// >maxbytes: important ! => possible if fd was not open !
		{
			*nrofbytesread = static_cast<unsigned int>(ret);
			return true;
		}
		else
		{
#ifdef __SERIALCOM_DEBUG__
			std::cout << "serialCOM::Read():   read(..) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
			return false;
		}
	}
	else
	{
		unsigned int rb_total = 0;
		unsigned int t_remaining_ms = 0;
		struct timespec t_current;
		struct timespec t_start;
		fd_set input;
		struct timeval sel_timeout;
		int n = 0;

		if (timeout > 0)
		{
			if (clock_gettime(CLOCK_REALTIME, &t_start) != 0)	// returns -1 on error, 0 on success
			{
#ifdef __SERIALCOM_DEBUG__
				std::cout << "serialCOM::Read():   clock_gettime(...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
				return false;
			}
			t_remaining_ms = timeout;
		}
		do
		{
			FD_ZERO(&input);
			FD_SET(fd, &input);
			// Wait for data:
			if (timeout > 0)
			{
				sel_timeout.tv_sec  = t_remaining_ms / 1000;
				sel_timeout.tv_usec = (t_remaining_ms % 1000)*1000;
				n = select(fd+1, &input, NULL, NULL, &sel_timeout);
			}
			else // wait indefinitely
				n = select(fd+1, &input, NULL, NULL, NULL);
			/* NOTE: ONLY ON LINUX, select() modifies timeout to reflect the time not slept */
			// See if there was an error, read available data:
			if (n < 0)
			{
#ifdef __SERIALCOM_DEBUG__
				std::cout << "serialCOM::Read():   select(...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
				return false;
			}
			else if (n == 0)
			{
#ifdef __SERIALCOM_DEBUG__
				std::cout << "serialCOM::Read():   TIMEOUT\n";
#endif
				break;
			}
			else if (FD_ISSET(fd, &input))
			{
				// Read available data:
				ret = read(fd, data+rb_total, maxbytes-rb_total);	// NOTE: returns immediately
				if ((ret >= 0) && (ret <= static_cast<int>(maxbytes-rb_total)))	// >maxbytes: important ! => possible if fd was not open !
					rb_total += ret;
				else
				{
					*nrofbytesread = 0;
#ifdef __SERIALCOM_DEBUG__
					std::cout << "serialCOM::Read():   read(..) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
					return false;
				}
			}
			if (timeout > 0) // && (rb_total < minbytes)) // FIXME
			{
				// Get current time, calculate remaining time:
				if (clock_gettime(CLOCK_REALTIME, &t_current) != 0)	// returns -1 on error, 0 on success
				{
#ifdef __SERIALCOM_DEBUG__
					std::cout << "serialCOM::Read():   clock_gettime(...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
					return false;
				}
				t_remaining_ms = timeout - ((t_current.tv_sec*1000 + t_current.tv_nsec/1000000) - (t_start.tv_sec*1000 + t_start.tv_nsec/1000000)); // NOTE: overflow possible !
			}
		} while (((timeout == 0) || ((t_remaining_ms > 0) && (t_remaining_ms <= timeout))) && (rb_total < minbytes)); // NOTE: 2nd check of t_remaining_ms: for detecting overflow
		*nrofbytesread = static_cast<unsigned int>(rb_total);
		return true;
	}
	/* NOTE: - we always return the received bytes even if we received less than minbytes (timeout)
	         - return value indicates error but not a timeout (can be checked by comparing minbytes and nrofbytesread) */
}


bool serialCOM::ClearSendBuffer()
{
	int cvTF = -1;
	if (portisopen)
		cvTF = tcflush(fd, TCOFLUSH);
	if (cvTF != -1)
		return true;
	else
	{
#ifdef __SERIALCOM_DEBUG__
		std::cout << "serialCOM::ClearSendBuffer(...):   tcflush(..., TCOFLUSH) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
		return false;
	}
}


bool serialCOM::ClearReceiveBuffer()
{
	int cvTF = -1;
	if (portisopen)
		cvTF = tcflush(fd, TCIFLUSH);
	if (cvTF != -1)
		return true;
	else
	{
#ifdef __SERIALCOM_DEBUG__
		std::cout << "serialCOM::ClearReceiveBuffer(...):   tcflush(..., TCIFLUSH) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
		return false;
	}
}


bool serialCOM::SendBreak(unsigned int duration_ms)
{
	int confirmSB = -1;	// 0 or -1
	if ((!portisopen) || (duration_ms < 1) || (duration_ms >= 32767))
		return false;
	breakset = true;
	// We have to do the timing on our own
	confirmSB = ioctl(fd, TIOCSBRK, 0);	// break ON
	if (confirmSB != -1)
	{
		usleep(1000*duration_ms);		// GLIBC uses select() here... Would that be more accurate ?
		confirmSB = ioctl(fd, TIOCCBRK, 0);	// break OFF
		if (confirmSB == -1)
		{
#ifdef __SERIALCOM_DEBUG__
			std::cout << "serialCOM::SendBreak(...):   ioctl(..., TIOCCBRK, ...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
			return false;	// WITH breakset STILL TRUE !
		}
	}
#ifdef __SERIALCOM_DEBUG__
	else
		std::cout << "serialCOM::SendBreak(...):   ioctl(..., TIOCSBRK, ...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
	breakset = false;
	if (confirmSB == -1)
		return false;
	return true;
}


bool serialCOM::SetBreak()
{
	int confirmIOCTL = -1;
	if (!portisopen) return false;
	confirmIOCTL = ioctl(fd, TIOCSBRK, 0);	// break ON
	if (confirmIOCTL == -1)
	{
#ifdef __SERIALCOM_DEBUG__
		std::cout << "serialCOM::SetBreak(...):   ioctl(..., TIOCSBRK, ...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
		return false;
	}
	breakset = true;
	return true;
}


bool serialCOM::ClearBreak()
{
	int confirmIOCTL = -1;
	if (!portisopen) return false;
	confirmIOCTL = ioctl(fd, TIOCCBRK, 0);	   // break OFF
	if (confirmIOCTL == -1)
	{
#ifdef __SERIALCOM_DEBUG__
		std::cout << "serialCOM::ClearBreak(...):   ioctl(..., TIOCCBRK, ...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
		return false;
	}
	breakset = false;
	return true;
}


bool serialCOM::BreakIsSet()
{
	return breakset;
}


bool serialCOM::GetNrOfBytesAvailable(unsigned int *nbytes)
{
	int bytes = 0;
	if (!portisopen) return false;
	if (ioctl(fd, FIONREAD, &bytes) != -1)
	{
		*nbytes = static_cast<unsigned int>(bytes);
		return true;
	}
	else
	{
		*nbytes = 0;
#ifdef __SERIALCOM_DEBUG__
		std::cout << "serialCOM::GetNrOfBytesAvailable(...):   ioctl(..., FIONREAD, ...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
		return false;
	}
}


bool serialCOM::SetControlLines(bool DTR, bool RTS)
{
	int linestatus = 0;
	int retIOCTL = -1;
	if (!portisopen) return false;
	retIOCTL = ioctl(fd, TIOCMGET, &linestatus);
	if (retIOCTL == -1)
	{
		linestatus |= TIOCM_ST;	// set seconary TX-line (only DB25) to 1/HIGH = idle state;
#ifdef __SERIALCOM_DEBUG__
		std::cout << "serialCOM::SetControlLines():   ioctl(..., TIOCMGET, ...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
	}
	if (DTR)
		linestatus |= TIOCM_DTR;	// "Ready"
	else
		linestatus &= ~TIOCM_DTR;	// "NOT Ready"
	if (RTS)
		linestatus |= TIOCM_RTS;	// "Request"
	else
		linestatus &= ~TIOCM_RTS;	// "NO Request"
	/* NOTE: lines are inverted. Set flag means line=0/low/"space" */
	retIOCTL = ioctl(fd, TIOCMSET, &linestatus);
#ifdef __SERIALCOM_DEBUG__
	if (retIOCTL == -1)
		std::cout << "serialCOM::SetControlLines():   ioctl(..., TIOCMSET, ...) failed with error " << errno << " " << strerror(errno) << "\n";
#endif
	return (retIOCTL != -1);
}

// PRIVATE

struct serialCOM::std_baudrate serialCOM::std_baudrates[] = {
	{50, B50},
	{75, B75},
	{110, B110},
	{134.5, B134},
	{150, B150},
	{200, B200},
	{300, B300},
	{600, B600},
	{1200, B1200},
	{1800, B1800},
	{2400, B2400},
	{4800, B4800},
#ifdef B7200
	{7200, B7200},
#endif
	{9600, B9600},
#ifdef B14400
	{14400, B14400},
#endif
	{19200, B19200},
#ifdef B28800
	{28800, B28800},
#endif
	{38400, B38400},
	{57600, B57600},
#ifdef B76800
	{76800, B76800},
#endif
	{115200, B115200},
#ifdef B230400
	{230400, B230400},
#endif
};
// B0 not used, because of Windows compatibility; B110, B134: divisor not unique


bool serialCOM::GetStdBaudRateDCBConst(double baudrate, speed_t *DCBbaudconst)
{
	for (unsigned int k=0; k<(sizeof(std_baudrates)/sizeof(std_baudrate)); k++)
	{
		if (std_baudrates[k].value == baudrate)
		{
			*DCBbaudconst = std_baudrates[k].constant;
			return true;
		}
	}
	return false;
}


speed_t serialCOM::GetNearestStdBaudrate(double selBaudrate)
{
	// Get nearest standard baud rate:
	speed_t nearestBaudrate = 0;

	unsigned int stdbaudelements = (sizeof(std_baudrates)/sizeof(std_baudrate));

	if (selBaudrate <= std_baudrates[0].value)
	{
		nearestBaudrate=std_baudrates[0].constant;
	}
	else if (selBaudrate >= std_baudrates[stdbaudelements-1].value)
	{
		nearestBaudrate=std_baudrates[stdbaudelements-1].constant;
	}
	else
	{
		double q2=0;
		double q1=0;
		for (unsigned int b=1; b<stdbaudelements; b++)
		{
			q2 = std_baudrates[b].value / selBaudrate;
			if (q2 >= 1)
			{
				// br[b-1] < baudrate < br[b]:
				q1 = std_baudrates[b-1].value / selBaudrate;
				// compare relative baud rate deviation, select baud rate:
				if ((q2-1) < (1-q1))
					nearestBaudrate=std_baudrates[b].constant;
				else
					nearestBaudrate=std_baudrates[b-1].constant;
				break;
			}
		}
	}
	return nearestBaudrate;
}
