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

struct app
{
	app()
		: m_cd(0), m_no_more_commands(false), m_programming_mode(false)
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
	
			if (cmd == 255)
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

		if (cmd_parser.size() != 4)
			throw std::runtime_error("Failed to identify the device on the other side of the comm line.");

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
		
	int run_command(std::string const & cmd, int argc, char * argv[])
	{
		if (cmd.empty())
		{
		}
		else if (cmd == ":chipid")
		{
			ensure_programming_mode();
			get_chip_def();
			print_chip_id(m_cd, &signature[0]);
		}
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
			
			std::string memory_type = argv[0];

			std::size_t memsize = 0;
			
			get_chip_def();
			if (m_cd != 0)
			{
				std::map<std::string, chipdef::memorydef>::const_iterator ci = m_cd->memories.find(memory_type);
				if (ci == m_cd->memories.end())
				{
					std::cerr << "error: the chip does not have this type of memory" << std::endl;
					return 4;
				}
				memsize = ci->second.size;
			}
			
			if (memsize != 0 && length + start > memsize)
			{
				std::cerr << "abort: the device doesn't have that much memory" << std::endl;
				return 5;
			}
			
			if (length == 0 && memsize != 0)
				length = memsize;
			
			if (length == 0)
			{
				std::cerr << "abort: specify the length of the memory block you want to read" << std::endl;
				return 6;
			}
			
			std::vector<uint8_t> buf;
			uint32_t out_address = start;
			
			int memid = 0;
			if (memory_type == "flash")
				memid = 1;
			else if (memory_type == "eeprom")
				memid = 2;
			else
			{
				std::cerr << "error: unknown memory type: " << memory_type << std::endl;
				return 2;
			}

			uint32_t processed = 0;
			while (processed < length)
			{
				uint32_t chunk = length - processed;
				if (chunk > 127)
					chunk = 127;

				uint32_t base = start + processed;
				send_packet(0x46, memid, base, base >> 8, base >> 16, base >> 24, chunk);
				receive_packet(4);
				buf.insert(buf.end(), cmd_parser.data(), cmd_parser.data() + cmd_parser.size());
				
				while (buf.size() > 16)
				{
					print_hex_line(out_address, &buf[0], 16);
					out_address += 16;
					buf.erase(buf.begin(), buf.begin() + 16);
				}
				
				processed += chunk;
			}
			print_hex_line(out_address, &buf[0], buf.size());
			std::cout << ":00000001FF" << std::endl;
		}
		else if (cmd == ":erase")
		{
			ensure_programming_mode();
			send_packet(0x60);
			receive_packet(6);
		}
		else if (cmd == ":write")
		{
			ensure_programming_mode();
			if (argc < 1)
			{
				std::cerr << "Usage: avricsp <dev> :write <memorytype> [<pagesize>]" << std::endl;
				return 0;
			}

			if (std::string("flash") != argv[0])
			{
				std::cerr << "error: only flash memory writing is currently supported." << std::endl;
				return 2;
			}

			std::size_t pagesize = 0;
			std::size_t memsize = 0;
			if (argc > 1)
				pagesize = boost::lexical_cast<std::size_t>(argv[1]);
			
			get_chip_def();
			if (m_cd != 0)
			{
				std::map<std::string, chipdef::memorydef>::const_iterator ci = m_cd->memories.find("flash");
				if (ci == m_cd->memories.end())
				{
					std::cerr << "error: the chip does not have this type of memory" << std::endl;
					return 4;
				}
				pagesize = ci->second.pagesize;
				memsize = ci->second.size;
			}
			
			if (pagesize == 0)
			{
				std::cerr << "abort: the chip is unknown, you must specify the size of memory pages" << std::cerr;
				return 2;
			}
			
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
			
			if (memsize != 0 && program.size() > memsize)
			{
				std::cerr << "abort: the program will not fit into the memory" << std::endl;
			}

			uint8_t send_buffer[18];
			send_buffer[0] = 0x80;
			std::vector<uint8_t> pagebuffer;
			pagebuffer.resize(pagesize);

			for (std::size_t i = 0; i < program.size(); i += pagesize)
			{
				std::size_t program_chunk_size = (std::min)(program.size() - i, pagesize);
				std::copy(program.begin() + i, program.begin() + i + program_chunk_size, pagebuffer.begin());
				std::fill(pagebuffer.begin() + program_chunk_size, pagebuffer.end(), 0xff);
				
				bool empty = true;
				for (std::size_t j = 0; empty && j < pagebuffer.size(); ++j)
				{
					if (pagebuffer[j] != 0xff)
						empty = false;
				}
				
				if (empty)
					continue;

				send_packet(0x55, 1, i, i >> 8, i >> 16, i >> 24);
				receive_packet(5);

				std::size_t address = 0;
				while (address < pagebuffer.size())
				{
					std::size_t chunk = pagebuffer.size() - address;
					if (chunk > 12)
						chunk = 12;
					
					std::size_t packet_size = 0;
					for (; packet_size < chunk; ++packet_size)
					{
						send_buffer[packet_size + 4] = pagebuffer[address+packet_size];
					}
					
					send_buffer[1] = 0x70 | (packet_size+2);
					send_buffer[2] = 1;
					send_buffer[3] = address/2;
					lc.write(send_buffer, send_buffer + packet_size + 4);
					receive_packet(7);

					address += chunk;
				}
				
				std::cout << "Writing page at address " << i << std::endl;
				send_packet(0x85, 1, i, i >> 8, i >> 16, i >> 24);
				receive_packet(8);
			}
		}
		else if (cmd == ":writefuses")
		{
			ensure_programming_mode();
			uint8_t fuse_bytes[4] = {};
			
			get_chip_def();
			std::copy(signature.begin() + 3, signature.begin() + 7, fuse_bytes);
			
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
					
					if (!m_cd)
					{
						std::cerr << "abort: the chip signature is unknown, specify fuses as a sequence of hexadecimal digits" << std::endl;
						return 8;
					}
					
					bool found = false;
					for (std::size_t j = 0; !found && j < m_cd->fuses.size(); ++j)
					{
						if (m_cd->fuses[j].name == name)
						{
							for (std::size_t k = 0; k < m_cd->fuses[j].bits.size(); ++k)
							{
								int bitno = m_cd->fuses[j].bits[k];
								int byteno = bitno / 8;
								bitno %= 8;
								
								if (byteno > 3)
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
				else if (arg.size() < 8)
				{
					bool next_high = true;
					int index = 0;
					for (std::size_t j = 0; j < arg.size(); ++j)
					{
						int value;
						if ('0' <= arg[j] && arg[j] <= '9')
							value = arg[j] - '0';
						else if ('a' <= arg[j] && arg[j] <= 'f')
							value = arg[j] - 'a' + 10;
						else if ('A' <= arg[j] && arg[j] <= 'F')
							value = arg[j] - 'A' + 10;
						else
						{
							std::cerr << "abort: invalid argument: " << arg << std::endl;
							return 7;
						}

						if (j % 2 == 0)
							fuse_bytes[j/2] = (fuse_bytes[j/2] & 0xf0) | (value << 4);
						else
							fuse_bytes[j/2] = (fuse_bytes[j/2] & 0x0f) | (value);
					}
				}
				else
				{
					std::cerr << "abort: invalid argument: " << arg << std::endl;
					return 7;
				}
			}
			
			std::cout << "current=";
			print_chip_fuses(m_cd, &signature[3]);
			std::cout << "new    =";
			print_chip_fuses(m_cd, fuse_bytes);
			
			if (!force && m_cd == 0)
			{
				std::cerr << "abort: cannot verify the safety of fuse values.\nVerify the fuse values manually and rerun the command with --force." << std::endl;
				return 10;
			}
			
			if (!force && !m_cd->is_value_safe(fuse_bytes, fuse_bytes + 4))
			{
				std::cerr << "abort: the new fuse values are unsafe. Rerun the command with --force." << std::endl;
				return 10;
			}
			
			send_packet(0x94, fuse_bytes[0], fuse_bytes[1], fuse_bytes[2], fuse_bytes[3]);
			receive_packet(9);
			m_cd = 0;
			get_chip_def();
			std::cout << "verify =";
			print_chip_fuses(m_cd, &signature[3]);
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
	
	void get_chip_def()
	{
		if (m_cd != 0)
			return;
		
		send_packet(0x30);
		receive_packet(3);
		if (cmd_parser.size() < 3)
			throw std::runtime_error("failed to read chip signature");
			
		signature.assign(cmd_parser.data(), cmd_parser.data() + 8);
		m_cd = find_chipdef((cmd_parser[0] << 16) | (cmd_parser[1] << 8) | (cmd_parser[2]), chipdefs);
	}
	
private:
	std::vector<uint8_t> m_id;

	std::vector<chipdef> chipdefs;
	std::vector<uint8_t> signature;
	comm lc;
	command_parser cmd_parser;
	chipdef const * m_cd;
	bool m_no_more_commands;

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
