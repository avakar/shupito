#ifdef WIN32
# include "windows_comm.hpp"
#else
# include "linux_comm.hpp"
#endif

#include <iostream>
#include <stdarg.h>
#include <iomanip>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <fstream>

#include "chipdefs.hpp"

struct hex_parse_error
	: std::runtime_error
{
	hex_parse_error(const char * message, int line)
		: std::runtime_error(format_message(message, line)), m_line(line)
	{
	}

	int line() const { return m_line; };

private:
	std::string format_message(const char * message, int line)
	{
		return (boost::format("%s (on line %i).") % message % line).str();
	}

	int m_line;
};

class command_parser
{
public:
	enum state_t { bad, ready, simple_command, header, st_data, st_chunked };

	command_parser()
		: m_state(ready), m_cmd(0), m_cmd_size(0)
	{
	}

	void clear()
	{
		m_state = ready;
	}

	uint8_t command() const { return m_cmd; }
	uint8_t size() const { return m_buffer.size(); }

	uint8_t push_data(uint8_t ch)
	{
		switch (m_state)
		{
		case ready:
			if (ch == 0x80)
			{
				m_state = header;
				break;
			}

			if (ch > 16)
			{
				m_buffer.clear();
				m_cmd = ch;
				m_state = simple_command;
				break;
			}

			m_state = bad;
			break;

		case simple_command:
			m_state = bad;
			break;

		case header:
			m_cmd = ch >> 4;
			if (m_cmd == 0x0f)
			{
				m_cmd = ch & 0x0f;
				m_state = st_chunked;
				m_remaining_chunk_length = 0;
				m_buffer.clear();
			}
			else
			{
				m_cmd_size = ch & 0x0f;
				m_buffer.clear();
				m_state = m_cmd_size == 0? ready: st_data;
			}
			break;

		case st_data:
			m_buffer.push_back(ch);
			m_state = m_cmd_size == m_buffer.size()? ready: st_data;
			break;
			
		case st_chunked:
			if (m_remaining_chunk_length == 0)
			{
				if (ch == 0)
					m_state = ready;
				else
					m_remaining_chunk_length = ch;
			}
			else
			{
				m_buffer.push_back(ch);
				--m_remaining_chunk_length;
			}
			break;

		default:
			;
		}

		if (m_state == bad)
			return 254;

		if (m_state == simple_command || m_state == ready)
			return m_cmd;

		return 255;
	}

	state_t state() const { return m_state; }
	uint8_t const * data() const { return &m_buffer[0]; }

	uint8_t operator[](uint8_t index) const { return m_buffer[index]; }

private:
	state_t m_state;

	uint8_t m_cmd;
	uint8_t m_cmd_size;

	std::vector<uint8_t> m_buffer;
	int m_remaining_chunk_length;
};

void print_hex_line(int address, uint8_t const * data, int length)
{
	uint8_t line[21];
	line[0] = length;
	line[1] = address >> 8;
	line[2] = address;
	line[3] = 0;
	for (int j = 0; j < length; ++j)
		line[j+4] = data[j];
	int sum = 0;
	for (int j = 0; j < length + 4; ++j)
		sum += line[j];
	line[length + 4] = 0x100 - (sum & 0xff);

	std::cout << ':';
	for (int j = 0; j < length + 5; ++j)
		std::cout << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << (int)line[j];
	std::cout << std::endl;
}

chipdef const * find_chipdef(uint32_t signature, std::vector<chipdef> const & chipdefs)
{
	std::string strsig = (boost::format("avr:%0.6x") % signature).str();
	for (std::size_t i = 0; i < chipdefs.size(); ++i)
	{
		if (chipdefs[i].signature == strsig)
		{
			return &chipdefs[i];
		}
	}
	return 0;
}

struct hex_output
{
	explicit hex_output(int out_address)
		: out_address(out_address)
	{
	}

	template <typename Iter>
	void operator()(Iter first, Iter last)
	{
		buf.insert(buf.end(), first, last);
				
		while (buf.size() > 16)
		{
			print_hex_line(out_address, &buf[0], 16);
			out_address += 16;
			buf.erase(buf.begin(), buf.begin() + 16);
		}
	}

	void close()
	{
		print_hex_line(out_address, &buf[0], buf.size());
		buf.clear();
		std::cout << ":00000001FF" << std::endl;
	}

	int out_address;
	std::vector<uint8_t> buf;
};

template <typename Container>
struct appender_functor
{
	appender_functor(Container & c)
		: m_c(c)
	{
	}

	template <typename Iter>
	void operator()(Iter first, Iter last)
	{
		m_c.insert(m_c.end(), first, last);
	}

	Container & m_c;
};

template <typename Container>
appender_functor<Container> appender(Container & c)
{
	return appender_functor<Container>(c);
}

struct app
{
	app()
		: m_no_more_commands(false), m_programming_mode(false)
	{
	}
	
	void send_packet(int cmd, ...)
	{
		uint8_t packet[18];
		packet[0] = 0x80;
		packet[1] = cmd;
		
		va_list args;
		va_start(args, cmd);
		for (int i = 0; i < (cmd & 0xf); ++i)
			packet[i+2] = va_arg(args, int);
		va_end(args);
		
		lc.write(packet, packet + 2 + (cmd & 0xf));
	}
	
	void receive_packet(int command)
	{
		for (;;)
		{
			uint8_t cmd = cmd_parser.push_data(lc.read());
			if (cmd == 254)
			{
				cmd_parser.clear();
				continue;
			}
	
			if (cmd == 255 || cmd == 0xe)
				continue;
	
			if (cmd != command)
				throw std::runtime_error("received invalid command");
	
			break;
		}
	}
	
	void ensure_programming_mode()
	{
		if (m_programming_mode)
			return;

		if (m_current_mode != 0x871e0846 && m_current_mode != 0xc2a4dd67)
			throw std::runtime_error("error: you must select the programming interface (use the :mode command)");

		send_packet(0x10);
		receive_packet(1);
		if (cmd_parser.size() < 1)
			throw std::runtime_error("error: corrupted response from the programmer\n"
				"failed to initialize the programming mode");
		
		switch (cmd_parser[0])
		{
		case 0:
			m_programming_mode = true;
			break;
		case 1:
			throw std::runtime_error("warning: the chip didn't respond, failed to initialize the programming mode");
		case 2:
			throw std::runtime_error("warning: the chip is too slow, failed to initialize the programming mode");
		default:
			throw std::runtime_error("warning: unknown error, failed to initialize the programming mode");
		}
	}

	void print_mode(uint32_t mode)
	{
		switch (mode)
		{
		case 0x871e0846:
			std::cout << "mode=avr_spi" << std::endl;
			break;
		case 0xc2a4dd67:
			std::cout << "mode=avrx_pdi" << std::endl;
			break;
		case 0:
			break;
		default:
			std::cout << "mode=<unknown>" << std::endl;
		}
	}

	int run(int argc, char * argv[])
	{
		if (argc < 2)
		{
			std::cout << "Usage: avricsp <device> [<command> [<arguments>]]..." << std::endl;
			return 0;
		}

		parse_chipdefs(embedded_chipdefs, chipdefs);
		
		lc.open(argv[1]);
		
		send_packet(0x00);
		receive_packet(0);
		if (cmd_parser.size() != 5 || cmd_parser[0] != 0x40 || cmd_parser[1] != 0xbd
			|| cmd_parser[2] != 0xe9 || cmd_parser[3] != 0x9f || cmd_parser[4] != 0xea)
		{
			throw std::runtime_error("Failed to identify the device on the other side of the comm line.");
		}

		// Get the list of functions that can be selected into the device
		send_packet(0x01, 0x02);
		receive_packet(0);
		if (cmd_parser.size() % 4 != 1 || cmd_parser[0] != 0x42)
			throw std::runtime_error("Invalid response.");

		for (size_t i = 1; i < cmd_parser.size(); i += 4)
		{
			uint32_t fn = cmd_parser[i] | (cmd_parser[i+1] << 8) | (cmd_parser[i+2] << 16) | (cmd_parser[i+3] << 24);
			if (fn == 0)
				continue;
			m_modes[fn] = i / 4 + 1;
		}

		// Get the current function.
		send_packet(0x01, 0x04);
		receive_packet(0);
		if (cmd_parser.size() != 5 || cmd_parser[0] != 0x44)
			throw std::runtime_error("Invalid response.");

		m_current_mode = cmd_parser[1] | (cmd_parser[2] << 8) | (cmd_parser[3] << 16) | (cmd_parser[4] << 24);
		this->print_mode(m_current_mode);
		m_id.assign(cmd_parser.data(), cmd_parser.data() + 4);

		int i = 2;
		int first = 2;
		std::string cmd;
		do
		{
			if (i == argc || argv[i][0] == ':')
			{
				int res = this->run_command(cmd, i - first, argv + first);
				if (m_no_more_commands && i < argc)
				{
					std::cerr << "warning: '" << cmd << "' is the last command executed" << std::endl;
					break;
				}
				
				if (res != 0)
					return res;
				if (i < argc)
					cmd = argv[i];
				first = i + 1;
			}
		}
		while (i++ < argc);

		if (cmd.empty())
			return this->run_command(":chipid", 0, 0);

		return 0;
	}
	
	void print_chip_fuses(chipdef const * cd, uint8_t const * fuses)
	{
		if (cd)
		{
			for (int i = 0; i < 4; ++i)
				std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)fuses[i];
			std::cout << " (" << cd->format_value(fuses, fuses + 4) << ")" << std::endl;
		}
		else
		{
			for (int i = 0; i < 4; ++i)
				std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)fuses[i];
			std::cout << std::endl;
		}
	}
	
	void print_chip_id(chipdef const * cd, uint8_t const * sig)
	{
		if (cd)
		{
			std::cout << "id=";
			for (int i = 0; i < 3; ++i)
				std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)sig[i];
			std::cout << " (" << cd->name;
			for (std::map<std::string, chipdef::memorydef>::const_iterator ci = cd->memories.begin(); ci != cd->memories.end(); ++ci)
				std::cout << ", " << ci->first << '=' << std::dec << ci->second.size;
			std::cout << ")\nfuses=";
			for (int i = 3; i < 7; ++i)
				std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)sig[i];
			std::cout << " (" << cd->format_value(sig + 3, sig + 7) << ")\ncal=";
			std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)sig[7] << std::endl;
		}
		else
		{
			std::cout << "id=";
			for (int i = 0; i < 3; ++i)
				std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)sig[i];
			std::cout << "\nfuses=";
			for (int i = 3; i < 7; ++i)
				std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)sig[i];
			std::cout << "\ncal=";
			std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)sig[7] << std::endl;
		}
	}

	template <typename Appendable>
	void read_memory(int memid, int start, int length, Appendable & buf)
	{
		int processed = 0;
		while (processed < length)
		{
			uint32_t chunk = length - processed;
			if (chunk > 127)
				chunk = 127;

			uint32_t base = start + processed;
			send_packet(0x46, memid, base, base >> 8, base >> 16, base >> 24, chunk);
			receive_packet(4);
			buf(cmd_parser.data(), cmd_parser.data() + cmd_parser.size());

			processed += chunk;
		}
	}

	template <typename Iter>
	void write_mempage(int memid, int start, Iter first, Iter last)
	{
		this->send_packet(0x65, memid, start, start >> 8, start >> 16, start >> 24);
		this->receive_packet(6);

		uint8_t packet[18];
		packet[0] = 0x80;
		packet[2] = memid;
		while (first != last)
		{
			int chunk = (std::min)(last - first, 14);

			packet[1] = 0x70 | (chunk + 1);

			for (int i = 0; i < chunk; ++i)
				packet[i+3] = *first++;

			lc.write(packet, packet + chunk + 3);
			this->receive_packet(7);
		}

		this->send_packet(0x85, memid, start, start >> 8, start >> 16, start >> 24);
		this->receive_packet(8);
	}

	template <typename Iter>
	void write_memory(chipdef::memorydef const & md, int start, Iter first, Iter last)
	{
		if (md.pagesize == 0)
		{
			this->write_mempage(md.memid, start, first, last);
		}
		else
		{
			BOOST_ASSERT(start % md.pagesize == 0);
			while (first != last)
			{
				int chunk = (std::min)((std::size_t)(last - first), md.pagesize);
				this->write_mempage(md.memid, start, first, first + chunk);
				first += chunk;
				start += chunk;
			}
		}
	}

	chipdef::memorydef const & get_memdef(chipdef const & cd, std::string const & memname)
	{
		std::map<std::string, chipdef::memorydef>::const_iterator it = cd.memories.find(memname);
		if (it == cd.memories.end())
			throw std::runtime_error("The required memory section could not be found: " + memname);

		return it->second;
	}

	std::vector<uint8_t> read_memory(chipdef const & cd, std::string const & memname)
	{
		chipdef::memorydef const & md = this->get_memdef(cd, memname);
		std::vector<uint8_t> fuses;
		this->read_memory(md.memid, 0, md.size, appender(fuses));
		return fuses;
	}

	void print_chipid(std::ostream & o, chipdef const & cd)
	{
		o << "id=";
		if (cd.name.empty())
			o << "<unknown>";
		else
			o << cd.name;

		o << " (" << cd.signature << ")\n";
	}

	template <typename Iter>
	void print_fuses(std::ostream & o, chipdef const & cd, Iter first, Iter last)
	{
		for (Iter p = first; p != last; ++p)
			o << std::hex << std::setw(2) << std::setfill('0') << (int)*p;
		o << " (" << cd.format_value(first, last) << ")" << std::endl;
	}

	int run_command(std::string const & cmd, int argc, char * argv[])
	{
		if (cmd.empty())
		{
		}
		else if (cmd == ":mode")
		{
			if (argc != 1)
			{
				std::cerr << "Usage: avricsp <dev> :mode (avr_spi | avrx_pdi)" << std::endl;
				return 0;
			}

			std::string new_mode_str = argv[0];
			uint32_t new_mode = 0;
			if (new_mode_str == "pdi" || new_mode_str == "avrx_pdi")
				new_mode = 0xc2a4dd67;
			else if (new_mode_str == "spi" || new_mode_str == "avr_spi")
				new_mode = 0x871e0846;
			else
				throw std::runtime_error("error: unknown mode, use \"avr_spi\" or \"avrx_pdi\"");

			if (m_modes.find(new_mode) == m_modes.end())
				throw std::runtime_error("error: the mode is not supported by the programmer");
			send_packet(0x02, 0x03, m_modes.find(new_mode)->second);
			receive_packet(0);
			if (cmd_parser.size() != 2 || cmd_parser[0] != 0x43)
				throw std::runtime_error("invalid response");
			if (cmd_parser[1] != 0)
				throw std::runtime_error("error: error occured while selecting a new mode into the programmer");
			m_current_mode = new_mode;
			this->print_mode(m_current_mode);
		}
		else if (cmd == ":chipid")
		{
			ensure_programming_mode();
			read_chip_def(m_cd2);
			print_chipid(std::cout, m_cd2);

			std::vector<uint8_t> fuses = read_memory(m_cd2, "fuses");
			std::cout << "fuses=";
			print_fuses(std::cout, m_cd2, fuses.begin(), fuses.end());
		}
#if 0
		else if (cmd == ":clock")
		{
			if (argc < 1)
			{
				std::cerr << "Usage: avricsp <dev> :clock <freq-in-kHz>" << std::endl;
				return 0;
			}
			
			int clock = boost::lexical_cast<int>(argv[0]);
			send_packet(0xa2, uint8_t(clock), uint8_t(clock >> 8));
			receive_packet(0x0a);
		}
#endif
		else if (cmd == ":read")
		{
			ensure_programming_mode();
			if (argc < 1)
			{
				std::cerr << "Usage: avricsp <dev> :read <memorytype> [<start>] [<length>]" << std::endl;
				return 0;
			}
			
			uint32_t start = 0;
			uint32_t length = 0;
			if (argc > 1)
				length = boost::lexical_cast<uint32_t>(argv[1]);
			if (argc > 2)
			{
				start = length;
				length = boost::lexical_cast<uint32_t>(argv[2]);
			}
			
			read_chip_def(m_cd2);

			std::string memory_type = argv[0];
			std::map<std::string, chipdef::memorydef>::const_iterator ci = m_cd2.memories.find(memory_type);
			if (ci == m_cd2.memories.end())
			{
				std::cerr << "error: the chip does not have this type of memory" << std::endl;
				return 4;
			}

			chipdef::memorydef const & md = ci->second;
			
			if (md.size != 0 && length + start > md.size)
			{
				std::cerr << "abort: the device doesn't have that much memory" << std::endl;
				return 5;
			}
			
			if (length == 0 && md.size != 0)
				length = md.size;
			
			if (length == 0)
			{
				std::cerr << "abort: specify the length of the memory block you want to read" << std::endl;
				return 6;
			}

			hex_output out(start);
			this->read_memory(md.memid, start, length, out);
			out.close();
		}
		else if (cmd == ":erase")
		{
			ensure_programming_mode();
			send_packet(0x50);
			receive_packet(5);
		}
		else if (cmd == ":write")
		{
			ensure_programming_mode();
			read_chip_def(m_cd2);
			chipdef::memorydef md = this->get_memdef(m_cd2, argv[0]);

			if (argc < 1)
			{
				std::cerr << "Usage: avricsp <dev> :write <memorytype> [<pagesize>]" << std::endl;
				return 0;
			}

			if (argc > 1)
				md.pagesize = boost::lexical_cast<std::size_t>(argv[1]);

			std::istream & fin = std::cin;

			std::vector<uint8_t> program;
			std::vector<uint8_t> rec_nums;
			std::string line;
			int lineno = 0;
			uint32_t base = 0;
			while (std::getline(fin, line))
			{
				++lineno;

				if (line[0] != ':')
				{
					std::cerr << "error: input file corrupt" << std::endl;
					return 3;
				}
				
				if (!line.empty() && line.end()[-1] == '\r')
					line.erase(line.end() - 1);
	
				if (line.empty())
					continue;
	
				if (line[0] != ':' || line.size() % 2 != 1)
					throw hex_parse_error("An error occured", lineno);
	
				rec_nums.clear();
				for (std::size_t offs = 1; offs < line.size(); offs += 2)
				{
					std::string digit(&line[offs], 2);
					uint8_t res = uint8_t(strtol(digit.c_str(), 0, 16));
					rec_nums.push_back(res);
				}
	
				int length = rec_nums[0];
				int address = rec_nums[1] * 0x100 + rec_nums[2];
				int rectype = rec_nums[3];
	
				if (length != rec_nums.size() - 5)
					throw hex_parse_error("Invalid record length specified", lineno);
	
				if (rectype == 2)
				{
					if (length != 2)
						throw hex_parse_error("Invalid type 2 record", lineno);
					base = (rec_nums[4] * 0x100 + rec_nums[5]) * 16;
					continue;
				}
	
				if (rectype == 1)
					break;
	
				if (rectype != 0)
					throw hex_parse_error("Invalid record type", lineno);
	
				for (int i = 0; i < length; ++i)
				{
					if (base + address + i >= int(program.size()))
						program.resize(base + address + i + 1, 0xff);
	
					uint8_t & c = program[base + address + i];
					if (c != 0xff)
						throw hex_parse_error("A memory location was defined twice", lineno);
	
					c = rec_nums[i + 4];
				}
			}
			
			if (md.size != 0 && program.size() > md.size)
			{
				std::cerr << "abort: the program will not fit in" << std::endl;
				return 4;
			}

			this->write_memory(md, 0, program.begin(), program.end());
		}
		else if (cmd == ":writefuses")
		{
			ensure_programming_mode();
			this->read_chip_def(m_cd2);

			std::vector<uint8_t> old_fuse_bytes = this->read_memory(m_cd2, "fuses");
			std::vector<uint8_t> fuse_bytes = old_fuse_bytes;

			bool force = false;
			for (int i = 0; i < argc; ++i)
			{
				std::string arg = argv[i];
				if (arg == "--force")
				{
					force = true;
					continue;
				}
				
				if (arg[0] == '+' || arg[0] == '-' || arg.find('=') != std::string::npos)
				{
					int value = 0;
					std::string name;
					if (arg[0] == '+' || arg[0] == '-')
					{
						name = arg.substr(1);
						value = (arg[0] == '+'? 0: 1);
					}
					else
					{
						std::size_t pos = arg.find('=');
						name = arg.substr(0, pos);
						value = boost::lexical_cast<int>(arg.substr(pos + 1));
					}
					
					bool found = false;
					for (std::size_t j = 0; !found && j < m_cd2.fuses.size(); ++j)
					{
						if (m_cd2.fuses[j].name == name)
						{
							for (std::size_t k = 0; k < m_cd2.fuses[j].bits.size(); ++k)
							{
								int bitno = m_cd2.fuses[j].bits[k];
								int byteno = bitno / 8;
								bitno %= 8;
								
								if (byteno >= fuse_bytes.size())
									continue;
								
								if (value & 1)
									fuse_bytes[byteno] |= (1<<bitno);
								else
									fuse_bytes[byteno] &= ~(1<<bitno);
								value >>= 1;
							}
							
							found = true;
						}
					}
					
					if (!found)
					{
						std::cerr << "abort: unknown fuse: " << name << std::endl;
						return 9;
					}
				}
				else
				{
					std::cerr << "abort: invalid argument: " << arg << std::endl;
					return 7;
				}
			}

			std::cout << "current=";
			this->print_fuses(std::cout, m_cd2, old_fuse_bytes.begin(), old_fuse_bytes.end());
			std::cout << "new    =";
			this->print_fuses(std::cout, m_cd2, fuse_bytes.begin(), fuse_bytes.end());
			
			if (!force && !m_cd2.is_value_safe(fuse_bytes.begin(), fuse_bytes.end()))
			{
				std::cerr << "abort: the new fuse values are unsafe. Rerun the command with --force." << std::endl;
				return 10;
			}

			chipdef::memorydef const & md = this->get_memdef(m_cd2, "fuses");
			this->write_memory(md, 0, fuse_bytes.begin(), fuse_bytes.end());

			fuse_bytes = this->read_memory(m_cd2, "fuses");
			std::cout << "verify =";
			this->print_fuses(std::cout, m_cd2, fuse_bytes.begin(), fuse_bytes.end());
		}
		else if (cmd == ":run")
		{
			send_packet(0x20);
			receive_packet(2);
			m_no_more_commands = true;
		}
		else
		{
			std::cerr << "error: unknown command, run without arguments to get help" << std::endl;
			return 4;
		}
		return 0;
	}

	void read_chip_def(chipdef & cd)
	{
		if (!cd.signature.empty())
			return;

		send_packet(0x30);
		receive_packet(3);
		if (cmd_parser.size() < 3)
			throw std::runtime_error("failed to read chip signature");
		cd.signature = m_current_mode == 0x871e0846? "avr:": "avrx:";
		for (size_t i = 0; i < cmd_parser.size(); ++i)
		{
			static char const hexdigits[] = "0123456789abcdef";
			cd.signature.push_back(hexdigits[(cmd_parser[i] >> 4) & 0xf]);
			cd.signature.push_back(hexdigits[cmd_parser[i] & 0xf]);
		}
		update_chipdef(chipdefs, cd);
	}
	
private:
	std::vector<uint8_t> m_id;

	std::map<uint32_t, uint32_t> m_modes;
	uint32_t m_current_mode;

	std::vector<chipdef> chipdefs;
	comm lc;
	command_parser cmd_parser;
	bool m_no_more_commands;

	chipdef m_cd2;

	bool m_programming_mode;
};

int main(int argc, char * argv[])
{
	try
	{
		app a;
		return a.run(argc, argv);
	}
	catch (std::exception const & e)
	{
		std::cerr << "error: " << e.what() << std::endl;
	}
	catch (...)
	{
		std::cerr << "unknown error" << std::endl;
	}
	
	return 1;
}
