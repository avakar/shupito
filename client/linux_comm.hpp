#ifndef LINUX_COMM_HPP
#define LINUX_COMM_HPP

#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <string>
#include <stdexcept>

class linux_comm
{
public:
	typedef uint8_t char_type;

	linux_comm()
		: f(-1)
	{
	}

	void open(std::string const & dev)
	{
		int f = ::open(dev.c_str(), O_RDWR | O_NOCTTY);
		if (f < 0)
			throw std::runtime_error("Failed to open the serial port.");

		termios config;
		tcgetattr(f, &config);
		config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
			INLCR | PARMRK | INPCK | ISTRIP | IXON);
		config.c_oflag = 0;
		config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
		config.c_cflag = config.c_cflag & ~(CSIZE | PARENB) | CS8;
		config.c_cc[VMIN]  = 0;
		config.c_cc[VTIME] = 0;
		cfsetispeed(&config, B38400);
		cfsetospeed(&config, B38400);
		tcsetattr(f, TCSAFLUSH, &config);

		fcntl(f, F_SETFL, fcntl(f, F_GETFL) | O_NONBLOCK);

		this->close();
		this->f = f;
	}

	~linux_comm()
	{
		this->close();
	}
	
	void close()
	{
		if (f != -1)
		{
			::close(f);
			f = -1;
		}
	}
	
	bool is_open() const
	{
		return f != -1;
	}

	void write(char_type const * first, char_type const * last)
	{
		fd_set fs;
		FD_ZERO(&fs);

		while (first != last)
		{
			FD_SET(f, &fs);
			select(f + 1, 0, &fs, 0, 0);
			int written = ::write(f, first, last - first);
			if (written < 0)
				throw std::runtime_error("Failed to write into the serial port.");
			first += written;
		}
	}

	char_type * read(char_type * first, char_type * last, int microsecs = 0)
	{
		fd_set fs;
		FD_ZERO(&fs);

		do
		{
			FD_SET(f, &fs);
			if (microsecs != 0)
			{
				timeval timeout = { 0, microsecs };
				if (select(f + 1, &fs, 0, 0, &timeout) < 1)
					break;
			}
			else
				select(f + 1, &fs, 0, 0, 0);

			int r = ::read(f, first, last - first);
			if (r < 0)
				throw std::runtime_error("failed to read from the serial port");
			first += r;
		}
		while (first != last);

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
		tcflush(f, TCIFLUSH);
	}

private:
	int f;
	
	linux_comm(linux_comm const &);
	linux_comm & operator=(linux_comm const &);
};

#endif