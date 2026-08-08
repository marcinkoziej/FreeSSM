# Spec: macOS `serialCOM` implementation (part of the Linux→macOS port)

## Goal

FreeSSM's `.pro` file already branches serial-port code by OS (`unix { src/linux/... }`,
`win32 { src/windows/... }`). This spec covers the third branch: a `macx { src/mac/... }`
implementation of `serialCOM`, the class that talks to the K-line/SSM interface cable's
serial port. Everything else needed for a macOS build (the `.pro` `macx {}` scope itself,
`TimeM`/`J2534_API` ports) is covered briefly at the end; this doc is primarily the
`serialCOM` spec because it's the only file that needs a real rewrite.

**Do not redesign the class.** The public interface, the semantics of every method, and the
general code structure/comment style must match `src/linux/serialCOM.h`/`.cpp` exactly —
only the OS primitives underneath change. Two consumers depend on exact behavior:
`src/SerialPassThroughDiagInterface.cpp` and `src/ATcommandControlledDiagInterface.cpp`,
neither of which should need to change.

## Files to create

- `src/mac/serialCOM.h`
- `src/mac/serialCOM.cpp`
- `src/mac/TimeM.h`, `src/mac/TimeM.cpp` — verbatim copies of `src/linux/TimeM.{h,cpp}`.
  That file only uses `clock_gettime(CLOCK_REALTIME, ...)`, which is plain POSIX and behaves
  identically on macOS (available since OS X 10.12). No changes needed beyond the copy and
  updating the file header comment from "(Linux version)" if present.
- `src/mac/J2534_API.h`, `src/mac/J2534_API.cpp` — verbatim copies of
  `src/linux/J2534_API.{h,cpp}`. This file only uses `dlopen`/`dlsym`/`dlclose` (`<dlfcn.h>`),
  which work identically on macOS. No J2534 vendor libraries exist for macOS in practice, so
  this is a dead code path there, but it must still compile.

## Required header (`src/mac/serialCOM.h`)

Public interface — copy verbatim from `src/linux/serialCOM.h`, unchanged:

```cpp
class serialCOM
{
public:
	serialCOM();
	~serialCOM();
	static std::vector<std::string> GetAvailablePorts();
	bool IsOpen();
	std::string GetPortname();
	bool GetPortSettings(double *baudrate, unsigned short *databits = NULL, char *parity = NULL, float *stopbits = NULL);
	bool SetPortSettings(double baudrate, unsigned short databits, char parity, float stopbits);
	bool OpenPort(std::string portname);
	bool ClosePort();
	bool Write(std::vector<char> data);
	bool Write(char *data, unsigned int datalen);
	bool Read(unsigned int minbytes, unsigned int maxbytes, unsigned int timeout, std::vector<char> *data);
	bool Read(unsigned int minbytes, unsigned int maxbytes, unsigned int timeout, char *data, unsigned int *nrofbytesread);
	bool ClearSendBuffer();
	bool ClearReceiveBuffer();
	bool SendBreak(unsigned int duration_ms);
	bool SetBreak();
	bool ClearBreak();
	bool BreakIsSet();
	bool GetNrOfBytesAvailable(unsigned int *nbytes);
	bool SetControlLines(bool DTR, bool RTS);
	// ... GetStdBaudRateDCBConst / GetNearestStdBaudrate as private helpers, see below
};
```

Private members — replace the Linux members with:

```cpp
private:
	struct std_baudrate {
		double value;
		speed_t constant;
	};

	bool GetStdBaudRateDCBConst(double baudrate, speed_t *DCBbaudconst);
	speed_t GetNearestStdBaudrate(double selBaudrate);

	int fd;
	bool portisopen;
	bool breakset;
	std::string currentportname;
	struct termios oldtio;          // backup of port settings (plain POSIX termios, NOT termios2)
	bool settingssaved;
	double currentbaudrate;         // NEW: tracks the baud rate actually applied by the last successful SetPortSettings() call — see "Baud rate strategy" below
	static struct std_baudrate std_baudrates[];
```

Drop entirely (Linux-only, no macOS equivalent, and not needed — see rationale below):
`struct serial_struct old_serdrvinfo`, `flag_async_low_latency_supported`,
`ioctl_tiocgserial_supported`, `ioctl_tiocsserial_supported`.

Includes:

```cpp
#include <cstring>      // memset(), strcpy()
#include <cmath>        // round()
#include <cstdlib>      // malloc()/free()
extern "C"
{
	#include <termios.h>        // struct termios, cfsetispeed/cfsetospeed/tcgetattr/tcsetattr/tcflush/tcdrain
	#include <fcntl.h>          // open(), fcntl()
	#include <dirent.h>         // opendir/readdir — see GetAvailablePorts()
	#include <sys/ioctl.h>      // ioctl() — TIOCEXCL/TIOCNXCL/TIOCMGET/TIOCMSET/TIOCSBRK/TIOCCBRK/FIONREAD
	#include <sys/select.h>     // select()
	#include <IOKit/serial/ioss.h>  // IOSSIOSPEED — custom/non-standard baud rates
	#include <limits.h>
	#include <unistd.h>         // usleep(), isatty(), close()
	#ifdef __SERIALCOM_DEBUG__
		#include <errno.h>
	#endif
}
#include <string>
#include <vector>
#include <ctime>
#ifdef __SERIALCOM_DEBUG__
	#include <iostream>
#endif
```

`<IOKit/serial/ioss.h>` requires linking the IOKit framework — see ".pro changes" below.

## Baud rate strategy (the one real design decision in this port)

This is the part that differs most from Linux, so read this section fully before writing
`GetPortSettings`/`SetPortSettings`.

**Background** (from Apple's own docs/sample code and community references — see Sources at
the bottom): macOS `termios` only exposes the standard POSIX `Bxxxx` rate constants through
`cfsetispeed()`/`cfsetospeed()`/`tcsetattr()`. Since Mac OS X 10.4 (Tiger), arbitrary/custom
baud rates are set with a private ioctl, `IOSSIOSPEED`, taking a `speed_t` (the raw numeric
baud rate, not a `Bxxxx` constant) — e.g. `ioctl(fd, IOSSIOSPEED, &speed)`. Important rules:

1. `IOSSIOSPEED` must be called **after** `tcsetattr()`, because `tcsetattr()` resets the
   baud rate — there's no way to encode a custom rate inside the `termios` struct itself
   (no `BOTHER`/`termios2` mechanism like Linux has).
2. Once `IOSSIOSPEED` has been used, the kernel-side `termios` state can no longer be
   reliably round-tripped — a subsequent `tcgetattr()` may not reflect the real applied rate.
   Reading the rate back via `cfgetispeed()` after setting a custom rate is **not reliable**.

**Consequence for this port:** don't try to read the baud rate back from the kernel at all.
Instead:

- Add the `double currentbaudrate` member described above.
- `SetPortSettings()`, on success, stores the numeric `baudrate` argument it just applied
  into `currentbaudrate` (do this only when `settingsvalid == true` and the ioctl/tcsetattr
  calls succeeded — mirror the Linux function's existing success/failure flow).
- `GetPortSettings()`'s baud-rate output (`*baudrate`) is simply `currentbaudrate` — no
  `cfgetispeed()` call, no `Bxxxx` decode table lookup needed for this field. Note this
  differs from `GetPortSettings()` on Linux (which decodes the live termios2 state) — the
  tracked-value approach is intentional and necessary here per the round-trip caveat above.
  Databits/parity/stopbits are unaffected by this and are still read back live from the
  kernel (see next section).
- This is safe for the app's own sanity check in `SerialPassThroughDiagInterface.cpp`
  (`SetPortSettings(1953, ...)` followed by a `GetPortSettings()` readback that must be
  within 3% of 1953) — it will trivially always match, same as it already trivially matches
  today on Windows (`DCB.BaudRate` readback there is also just the value the driver reports
  it accepted, not a hardware measurement).

**Setting the rate in `SetPortSettings()`:**

- Build a small static table `std_baudrates[]` (same idea as the Linux one), but populated
  with whichever `Bxxxx` constants actually exist in macOS's `<sys/termios.h>` — at minimum
  `B50, B75, B110, B134 (B134 is 134.5 baud, keep the same 134.5 mapping as Linux),
  B150, B200, B300, B600, B1200, B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200`.
  Check the SDK for higher constants (`B230400` and possibly BSD-only extras like `B7200`,
  `B14400`, `B28800`, `B76800`) and include them guarded with `#ifdef Bxxxxx` exactly like
  the Linux file already does for its own higher constants — don't assume the exact set,
  verify against the SDK header (`grep 'define.*B[0-9]' $(xcrun --show-sdk-path)/usr/include/sys/termios.h`).
- `GetStdBaudRateDCBConst()` / `GetNearestStdBaudrate()`: same logic/signatures as Linux,
  just operating on this mac table. Keep both — `GetNearestStdBaudrate()` is unused for now
  since the fallback-to-nearest-standard-rate-on-failure logic doesn't apply the same way
  (see below), but keep it for parity/possible future use, or drop it if truly dead —
  your call, but if you drop it, also drop the now-unreferenced Linux fallback comment.
- If the requested `baudrate` matches a table entry exactly: call
  `cfsetispeed(&newtio, constant)` and `cfsetospeed(&newtio, constant)` with the matched
  constant, then `tcsetattr(fd, TCSANOW, &newtio)`. No `IOSSIOSPEED` call needed.
- If it does not match (the common case for this app — e.g. 1953, 15625, 10400 baud): set
  `cfsetispeed`/`cfsetospeed` to any placeholder standard rate (`B9600` is fine, it's
  discarded), call `tcsetattr(fd, TCSANOW, &newtio)` first, **then**
  `speed_t customspeed = static_cast<speed_t>(round(baudrate)); ioctl(fd, IOSSIOSPEED, &customspeed)`.
  If the ioctl fails, `SetPortSettings()` returns `false` (there's no meaningful "fall back to
  nearest standard rate" recovery here the way Linux does with its `BOTHER` retry — a failed
  `IOSSIOSPEED` on macOS generally means the driver genuinely can't do arbitrary rates, and
  silently substituting a wrong rate would break protocol timing without the caller knowing).
- Do **not** port any of `TIOCGSERIAL`/`TIOCSSERIAL`/`ASYNC_SPD_CUST`/`custom_divisor`/
  `ASYNC_LOW_LATENCY` logic. That's all Linux-specific serial-driver-internals tuning with no
  macOS equivalent, and none of it is required for correct operation — see next section.

## Everything else: mostly a direct POSIX port, not a rewrite

Aside from the baud rate handling above, `struct termios` (macOS) is API-compatible with the
subset of `struct termios2` (Linux) fields this code actually uses for databits/parity/
stopbits/control flags (`c_cflag`, `c_lflag`, `c_iflag`, `c_oflag`, `c_cc[]`,
`CS5..CS8, PARENB, PARODD, CSTOPB, CLOCAL, CREAD, IGNPAR, IGNBRK, NOFLSH, OPOST, VMIN, VTIME`
are all standard POSIX and identical on both). `CMSPAR` (mark/space parity, used for parity
`'M'`/`'S'`) is a Linux-only non-POSIX extension and is **not** defined on macOS — the Linux
code already wraps that logic in `#ifdef CMSPAR`/`#endif`, so on macOS those branches simply
compile out and `'M'`/`'S'` parity requests correctly fall through to `settingsvalid = false`.
No change needed there — just port the surrounding code as-is with `#ifdef CMSPAR` intact.

Go through `src/linux/serialCOM.cpp` function by function:

| Function | Porting instructions |
|---|---|
| `GetAvailablePorts()` | Rewrite the enumeration only — see dedicated section below. |
| `IsOpen()` | Verbatim port, no OS-specific code. |
| `GetPortname()` | Verbatim port. |
| `GetPortSettings()` | Baud rate: return `currentbaudrate` directly (see strategy above), skip the whole `TCGETS2`/`BOTHER`/`TIOCGSERIAL` baud-decode block. Databits/parity/stopbits: same decode logic as Linux, but call `tcgetattr(fd, &currenttio)` (returns `-1`/`errno` on failure, same as the ioctl did) instead of `ioctl(fd, TCGETS2, &currenttio)`, and use `struct termios currenttio` instead of `struct termios2`. |
| `SetPortSettings()` | Baud rate: per strategy above. Databits/parity/stopbits/control-flag setup (`c_cflag`, `c_lflag`, `c_iflag`, `c_oflag`, `c_cc[VMIN]/[VTIME]`): port verbatim, same bit logic. Apply with `tcsetattr(fd, TCSANOW, &newtio)` instead of `ioctl(fd, TCSETS2, &newtio)`; check return `== 0` for success (`ioctl`/`tcsetattr` both return `-1` on error, so the success check `cIOCTL != -1` becomes `cTCSETATTR == 0`, keep behavior equivalent). Drop the entire `TIOCGSERIAL`/`TIOCSSERIAL`/`new_serdrvinfo` block (start of function) and the retry-with-`TIOCSSERIAL`-fallback block (near the end) — none of it applies on macOS. Remember: if baud rate is non-standard, call `ioctl(fd, IOSSIOSPEED, &customspeed)` **after** the `tcsetattr()` call, not before. |
| `OpenPort()` | `open(portname.c_str(), O_RDWR \| O_NOCTTY \| O_NDELAY)` — same flags, all defined on macOS. `ioctl(fd, TIOCEXCL, NULL)` — same, available on macOS/BSD (exclusive-open lock). `fcntl(fd, F_SETFL, FNDELAY)` — same. Save old settings: `tcgetattr(fd, &oldtio)` instead of `ioctl(fd, TCGETS2, &oldtio)`. Drop the entire `TIOCGSERIAL`/driver-info/`ASYNC_LOW_LATENCY` block — there is no macOS equivalent and it is not required (see note below). Flush buffers: `tcflush(fd, TCIOFLUSH)` (POSIX function) instead of `ioctl(fd, TCFLSH, TCIOFLUSH)` — macOS does not have the Linux `TCFLSH`/`TCIFLUSH`/`TCOFLUSH`/`TCIOFLUSH` ioctl numbering, use the portable `tcflush()` function instead, which takes the same `TCIFLUSH`/`TCOFLUSH`/`TCIOFLUSH` constants (they exist as POSIX `int` constants on macOS too, just consumed differently). Rest of the function (call `SetPortSettings(9600, 8, 'N', 1)`, then `SetControlLines(true, true)`) ports verbatim. |
| `ClosePort()` | Clear break: `ioctl(fd, TIOCCBRK, 0)` — same, available on macOS. Flush: `tcflush(fd, TCIOFLUSH)` instead of the `ioctl(..., TCFLSH, ...)`. Drop the `TIOCSSERIAL` restore block. Restore old settings: `tcsetattr(fd, TCSANOW, &oldtio)` instead of `ioctl(fd, TCSETS2, &oldtio)`. Unlock: `ioctl(fd, TIOCNXCL, NULL)` — same. `close(fd)` — same. Drop cleanup of the removed `ioctl_tiocgserial_supported`/etc. members; do reset `currentbaudrate` if you want (not strictly required, `portisopen` guards its use). |
| `Write(vector<char>)` | Verbatim port (delegates to the other overload). |
| `Write(char*, len)` | Break-clear via `ioctl(fd, TIOCCBRK, 0)` — same. `write(fd, data, datalen)` — same. Replace `ioctl(fd, TCSBRK, 1)` ("wait until transmitted", a Linux-specific `tcdrain()` reimplementation) with the actual POSIX function: `tcdrain(fd)` (returns `0`/`-1`, same success check pattern). |
| `Read(..., vector<char>*)` | Verbatim port (delegates to the other overload). |
| `Read(..., char*, uint*)` | **Verbatim port, no changes needed.** This function only uses `read()`, `select()`, `FD_ZERO`/`FD_SET`/`FD_ISSET`, and `clock_gettime(CLOCK_REALTIME, ...)` — all plain POSIX, identical on macOS. Note the existing Linux comment "`ONLY ON LINUX, select() modifies timeout to reflect the time not slept`" — the code already does NOT rely on that behavior (it recomputes `t_remaining_ms` itself via `clock_gettime()` every loop iteration), so this function needs zero logic changes for macOS despite that comment. |
| `ClearSendBuffer()` | `tcflush(fd, TCOFLUSH)` instead of `ioctl(fd, TCFLSH, TCOFLUSH)`. |
| `ClearReceiveBuffer()` | `tcflush(fd, TCIFLUSH)` instead of `ioctl(fd, TCFLSH, TCIFLUSH)`. |
| `SendBreak(duration_ms)` | Simplify: macOS has no `TCSBRK`/`TCSBRKP` ioctls (those are Linux/SysV termio numbering with Linux-specific semantics for the argument). Drop the `duration_ms == 250` special case and the `multiple-of-100ms` special case entirely. Always use the Linux code's third branch ("we have to do the timing on our own") for **all** durations: `ioctl(fd, TIOCSBRK, 0)` (break ON, available on macOS), `usleep(1000*duration_ms)`, `ioctl(fd, TIOCCBRK, 0)` (break OFF). Same `breakset`/return-value bookkeeping as that branch already has. |
| `SetBreak()` | Verbatim port — `ioctl(fd, TIOCSBRK, 0)` is available on macOS. |
| `ClearBreak()` | Verbatim port — `ioctl(fd, TIOCCBRK, 0)` is available on macOS. |
| `BreakIsSet()` | Verbatim port. |
| `GetNrOfBytesAvailable()` | Verbatim port — `ioctl(fd, FIONREAD, &bytes)` is available on macOS/BSD. |
| `SetControlLines()` | Verbatim port — `TIOCMGET`, `TIOCMSET`, `TIOCM_DTR`, `TIOCM_RTS`, `TIOCM_ST` all exist on macOS/BSD with the same meaning. |
| `GetStdBaudRateDCBConst()` / `GetNearestStdBaudrate()` | Same logic as Linux, operating on the mac `std_baudrates[]` table described above. |

### Why dropping `TIOCGSERIAL`/`TIOCSSERIAL`/`ASYNC_LOW_LATENCY` is correct, not a shortcut

This isn't skipped functionality — it's a Linux-specific workaround for particular UART/
USB-serial driver quirks (custom divisor programming for non-standard baud rates on old
drivers, and requesting reduced interrupt-coalescing latency from the Linux USB-serial
subsystem). macOS's IOKit-based USB-serial drivers don't expose or need either concept:
non-standard baud rates go through `IOSSIOSPEED` instead (a full replacement, not a
degraded one), and there's no macOS equivalent of "low latency mode" to request because the
driver model is different. Porting this block would mean calling ioctls macOS doesn't define.

## `GetAvailablePorts()`

Recommended approach for v1 — a `/dev` directory scan, matching the existing code's style
and requiring no new frameworks or `.pro` linking changes:

```cpp
std::vector<std::string> serialCOM::GetAvailablePorts()
{
	std::vector<std::string> portlist(0);
	int testfd = -1;
	char ffn[256] = "";
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
```

(Add `#include <algorithm>` for `std::sort` — the Linux version doesn't sort, but sorting
gives a stable/predictable order in the port-selection UI, matching what the Windows version
already does; harmless either way, include it for consistency.)

This deliberately does not exclude `/dev/cu.Bluetooth-Incoming-Port` or similar virtual
entries some macOS versions expose — if that shows up as noise during testing, filtering it
out by name is a reasonable follow-up but not a v1 requirement.

**Not required for v1, optional future enhancement:** a proper IOKit-based enumeration via
`IOServiceMatching(kIOSerialBSDServiceValue)` / `IOServiceGetMatchingServices()`, reading
`kIOCalloutDeviceKey` for each matched service (headers: `<IOKit/IOKitLib.h>`,
`<IOKit/serial/IOSerialKeys.h>`). This gives richer metadata (vendor/product strings) for a
nicer device picker, at the cost of pulling in `IOKit.framework` and `CoreFoundation.framework`
plus more code. Skip for the initial port; the `/dev/cu.*` scan is sufficient for the app to
function.

## `.pro` file changes (context, not this file's main deliverable)

Add a `macx {}` scope parallel to the existing `unix {}`/`win32 {}` ones:

```qmake
macx {
       DEPENDPATH += src/mac
       INCLUDEPATH += src/mac
       HEADERS += src/mac/serialCOM.h \
                  src/mac/TimeM.h \
                  src/mac/J2534_API.h
       SOURCES += src/mac/serialCOM.cpp \
                  src/mac/TimeM.cpp \
                  src/mac/J2534_API.cpp
       LIBS += -ldl -framework IOKit
}
```

And change the existing `unix { ... }` block (which currently unconditionally pulls in
`src/linux/*` for every Unix-like OS including macOS, and links `-lrt` which doesn't exist
on macOS) to `linux { ... }` instead of `unix { ... }`, keeping `LIBS += -ldl -lrt` there
unchanged. Also guard `QMAKE_CXXFLAGS += -fno-gcse` (a GCC-only flag, worked around a GCC≥4.2
codegen bug) with `!macx:` since Clang doesn't recognize it.

This `.pro` restructuring is **not** part of what should be handed to the implementer for
this spec — call it out separately, it's a small mechanical change to a file the serialCOM
implementer doesn't need to touch to write `src/mac/serialCOM.cpp` itself, but the build
won't work without it.

## Acceptance criteria

1. `src/mac/serialCOM.{h,cpp}`, `src/mac/TimeM.{h,cpp}`, `src/mac/J2534_API.{h,cpp}` exist
   and compile cleanly on macOS with Clang (`-std=c++17`, no `-fno-gcse`).
2. Public interface of `serialCOM` is identical to `src/linux/serialCOM.h` (same method
   signatures, same class name, same header guard convention) — callers in
   `src/SerialPassThroughDiagInterface.cpp`/`src/ATcommandControlledDiagInterface.cpp`
   require zero changes.
3. `GetAvailablePorts()` returns real `/dev/cu.*` device paths when a USB-serial adapter is
   plugged in (manually verify with `ls /dev/cu.*` before/after plugging in a cable).
4. `OpenPort()` → `SetPortSettings(1953, 8, 'E', 1)` → `GetPortSettings(&baudrate)` round
   trip: `baudrate` must equal exactly `1953` (per the tracked-value strategy above), so the
   existing 3%-tolerance check in `SerialPassThroughDiagInterface::connect()` passes.
5. `SetPortSettings(4800, 8, 'N', 1)` succeeds (this is the app's fallback/reset path, uses a
   standard baud rate — sanity-checks the `cfsetispeed`/`cfsetospeed` standard-rate path
   independent of `IOSSIOSPEED`).
6. If hardware is available: end-to-end smoke test — open the port, send/receive a few
   bytes, confirm `Read()` respects `timeout`/`minbytes`/`maxbytes` semantics, confirm
   `SendBreak()` produces a break of roughly the requested duration.
7. `__SERIALCOM_DEBUG__` build (uncomment the `#define` in the header, or build with
   `DEFINES += __SERIALCOM_DEBUG__`) compiles and produces debug output in the same style as
   the Linux file (same message prefixes, same `errno`/`strerror()` reporting pattern).

## Sources consulted for the macOS-specific APIs

- [BSD and macOS Compatibility | npat-efault/picocom | DeepWiki](https://deepwiki.com/npat-efault/picocom/5.1-bsd-and-macos-compatibility) — `IOSSIOSPEED` behavior, ordering relative to `tcsetattr()`, round-trip caveat.
- [serialport-rs NOTES.md](https://github.com/serialport/serialport-rs/blob/main/NOTES.md) — cross-platform serial library's own notes on macOS custom baud rate handling.
- [Observe serial ports on macOS — kyryl horbushko](https://khorbushko.github.io/article/2021/05/05/observe-serial-ports-on-macOS.html) — IOKit enumeration approach (for the optional future enhancement).
- [Apple SerialPortSample (archived sample code)](https://developer.apple.com/library/archive/samplecode/SerialPortSample/Listings/SerialPortSample_SerialPortSample_c.html) — canonical `IOServiceMatching(kIOSerialBSDServiceValue)` enumeration pattern.
- [IOKit forum: detecting BSD name for USB serial device](https://developer.apple.com/forums/thread/113808) — `kIOCalloutDeviceKey` usage.
