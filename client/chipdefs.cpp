#include "chipdefs.hpp"
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lexical_cast.hpp>

const std::string embedded_chipdefs =
	"atmega48 avr:1e9205 flash=4096:64,eeprom=256:4 cksel:8,9,10,11 sut:12,13 ckout:14 ckdiv8:15 bodlevel:16,17,18 eesave:19"
		" wdton:20 spien:21:0 dwen:22 rstdisbl:23:1 selfprgen:24\n"
	"atmega168 avr:1e9406 flash=16384:128,eeprom=512:4"
		" lb:0,1 blb0:2,3 blb1:4,5 cksel:8,9,10,11 sut:12,13 ckout:14 ckdiv8:15 bodlevel:16,17,18 eesave:19"
		" wdton:20 spien:21:0 dwen:22 rstdisbl:23:1 bootrst:24 bootsz:25,26\n"
	"atmega128 avr:1e9702 flash=131072:256,eeprom=4096"
		" lb:0,1 blb0:2,3 blb1:4,5 cksel:8,9,10,11 sut:12,13 boden:14 bodlevel:15"
		" bootrst:16 bootsz:17,18 eesave:19 ckopt:20 spien:21 jtagen:22 ocden:23 wdton:24 m103c:25\n"
	"atxmega64a avr:1e9441 flash=69632:256,eeprom=4096\n"
	;
	
void parse_chipdefs(std::string const & strdefs, std::vector<chipdef> & res)
{
	std::stringstream ss(strdefs);
	std::string line;
	
	while (std::getline(ss, line))
	{
		using namespace boost::lambda;

		std::vector<std::string> tokens;
		boost::algorithm::split(tokens, line, _1 == ' ');
		
		if (tokens.size() < 3)
			continue;

		chipdef def;
		def.name = tokens[0];
		def.signature = tokens[1];

		std::vector<std::string> memories;
		boost::algorithm::split(memories, tokens[2], _1 == ',');
		for (std::size_t i = 0; i < memories.size(); ++i)
		{
			std::vector<std::string> memory_tokens;
			boost::algorithm::split(memory_tokens, memories[i], _1 == '=');
			if (memory_tokens.size() != 2)
				continue;
			std::vector<std::string> mem_size_tokens;
			boost::algorithm::split(mem_size_tokens, memory_tokens[1], _1 == ':');
			chipdef::memorydef memdef;
			memdef.size = boost::lexical_cast<int>(mem_size_tokens[0]);
			if (mem_size_tokens.size() > 1)
				memdef.pagesize = boost::lexical_cast<int>(mem_size_tokens[1]);
			else
				memdef.pagesize = 0;
			def.memories[memory_tokens[0]] = memdef;
		}

		for (std::size_t i = 3; i < tokens.size(); ++i)
		{
			std::vector<std::string> token_parts;
			boost::algorithm::split(token_parts, tokens[i], _1 == ':');

			if (token_parts.size() != 2 && token_parts.size() != 3)
				continue;

			std::vector<std::string> bit_numbers;
			boost::algorithm::split(bit_numbers, token_parts[1], _1 == ',');

			chipdef::fuse f;
			f.name = token_parts[0];
			for (std::size_t j = 0; j < bit_numbers.size(); ++j)
				f.bits.push_back(boost::lexical_cast<int>(bit_numbers[j]));
				
			if (token_parts.size() == 3)
			{
				boost::algorithm::split(bit_numbers, token_parts[2], _1 == ',');
				for (std::size_t j = 0; j < bit_numbers.size(); ++j)
					f.values.push_back(boost::lexical_cast<int>(bit_numbers[j]));
			}

			def.fuses.push_back(f);
		}
		
		res.push_back(def);
	}
}

std::string chipdef::format_value(uint8_t const * first, uint8_t const * last) const
{
	std::string res;
	
	for (std::size_t i = 0; i < fuses.size(); ++i)
	{
		if (i > 0)
			res += ' ';

		int fusevalue = 0;
		for (std::size_t j = fuses[i].bits.size(); j > 0; --j)
		{
			int bitno = fuses[i].bits[j-1];
			int byteno = bitno / 8;
			bitno %= 8;
			
			if (byteno >= last - first)
				continue;
				
			fusevalue = (fusevalue << 1) | !!(first[byteno] & (1<<bitno));
		}
		
		if (fuses[i].bits.size() == 1)
			res += (fusevalue == 0? '+': '-');

		res += fuses[i].name;

		if (fuses[i].bits.size() != 1)
		{
			res += '=';
			res += boost::lexical_cast<std::string>(fusevalue);
		}

		if (fuses[i].values.size() != 0)
		{
			if (std::find(fuses[i].values.begin(), fuses[i].values.end(), fusevalue) == fuses[i].values.end())
			res += '!';
		}
	}
	
	return res;
}

bool chipdef::is_value_safe(uint8_t const * first, uint8_t const * last) const
{
	for (std::size_t i = 0; i < fuses.size(); ++i)
	{
		int fusevalue = 0;
		for (std::size_t j = fuses[i].bits.size(); j > 0; --j)
		{
			int bitno = fuses[i].bits[j-1];
			int byteno = bitno / 8;
			bitno %= 8;
			
			if (byteno >= last - first)
				continue;
				
			fusevalue = (fusevalue << 1) | !!(first[byteno] & (1<<bitno));
		}
		
		if (fuses[i].values.size() != 0)
		{
			if (std::find(fuses[i].values.begin(), fuses[i].values.end(), fusevalue) == fuses[i].values.end())
				return false;
		}
	}
	
	return true;
}
