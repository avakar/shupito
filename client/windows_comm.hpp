#ifndef WINDOWS_COMM_HPP
#define WINDOWS_COMM_HPP

#include <Windows.h>
#include <string>
#include <stdexcept>
#include <stdint.h>

class comm
{
public:
	typedef uint8_t char_type;

	comm()
		: m_hFile(INVALID_HANDLE_VALUE)
	{
		m_hEvent = ::CreateEventA(0, FALSE, FALSE, 0);
		if (!m_hEvent)
			throw std::runtime_error("Failed to create a synchronization object.");
	}

	void open(std::string const & dev)
	{
		HANDLE hFile = ::CreateFileA(dev.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);
		if (hFile == INVALID_HANDLE_VALUE)
			throw std::runtime_error("Failed to open the serial port.");

		DCB dcb = { sizeof dcb };
		dcb.BaudRate = 38400;
		dcb.fBinary = TRUE;
		dcb.ByteSize = 8;
		SetCommState(hFile, &dcb);

		COMMTIMEOUTS ct = {};
		SetCommTimeouts(hFile, &ct);

		this->close();
		m_hFile = hFile;
	}

	~comm()
	{
		this->close();
		::CloseHandle(m_hEvent);
	}
	
	void close()
	{
		if (m_hFile != INVALID_HANDLE_VALUE)
		{
			::CloseHandle(m_hFile);
			m_hFile = 0;
		}
	}
	
	bool is_open() const
	{
		return m_hFile != INVALID_HANDLE_VALUE;
	}

	void write(char_type const * first, char_type const * last)
	{
		while (first != last)
		{
			OVERLAPPED o = {};
			if (!::WriteFile(m_hFile, first, last - first, 0, &o) && GetLastError() != ERROR_IO_PENDING)
				throw std::runtime_error("Failed to write to the COM port.");

			DWORD dwWritten;
			if (!::GetOverlappedResult(m_hFile, &o, &dwWritten, TRUE))
				throw std::runtime_error("Failed to write to the COM port.");
			first += dwWritten;
		}
	}

	char_type * read(char_type * first, char_type * last, int microsecs = 0)
	{
		DWORD dwTimeBase = GetTickCount();
		DWORD dwWaitTime = microsecs / 1000;

		while (first != last)
		{
			OVERLAPPED o = {};
			o.hEvent = m_hEvent;
			if (!::ReadFile(m_hFile, first, last - first, 0, &o))
			{
				if (GetLastError() == ERROR_IO_PENDING)
				{
					DWORD dwElapsedTime = GetTickCount() - dwTimeBase;
					if (::WaitForSingleObject(m_hEvent, microsecs == 0? INFINITE: dwElapsedTime < dwWaitTime? dwWaitTime - dwElapsedTime: 0) == WAIT_TIMEOUT)
						::CancelIo(m_hFile);
				}
				else
					throw std::runtime_error("Failed to read from the COM port.");
			}

			DWORD dwRead;
			if (!::GetOverlappedResult(m_hFile, &o, &dwRead, FALSE))
				throw std::runtime_error("Failed to read from the COM port.");

			if (dwRead == 0)
				break;

			first += dwRead;
		}
		return first;
	}
	
	char_type read()
	{
		char_type r;
		this->read(&r, &r + 1);
		return r;
	}

	void rx_clear()
	{
		::PurgeComm(m_hFile, PURGE_RXCLEAR);
	}

private:
	HANDLE m_hFile;
	HANDLE m_hEvent;
	
	comm(comm const &);
	comm & operator=(comm const &);
};

#endif