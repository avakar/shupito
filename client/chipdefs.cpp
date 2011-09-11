#include "chipdefs.hpp"
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lexical_cast.hpp>

const std::string embedded_chipdefs =
	// lock bits, low fuses, high fuses, extended fuses
	"atmega48 avr:1e9205 flash=4096:64,eeprom=256:4 cksel:8,9,10,11 sut:12,13 ckout:14 ckdiv8:15 bodlevel:16,17,18 eesave:19"
		" wdton:20 spien:21:0 dwen:22 rstdisbl:23:1 selfprgen:24\n"
	"atmega168 avr:1e9406 flash=16384:128,eeprom=512:4"
		" lb:0,1 blb0:2,3 blb1:4,5 cksel:8,9,10,11 sut:12,13 ckout:14 ckdiv8:15 bodlevel:16,17,18 eesave:19"
		" wdton:20 spien:21:0 dwen:22 rstdisbl:23:1 bootrst:24 bootsz:25,26\n"
	"atmega128 avr:1e9702 flash=131072:256,eeprom=4096"
		" lb:0,1 blb0:2,3 blb1:4,5 cksel:8,9,10,11 sut:12,13 boden:14 bodlevel:15"
		" bootrst:16 bootsz:17,18 eesave:19 ckopt:20 spien:21:0 jtagen:22 ocden:23 wdton:24 m103c:25\n"

	// the datasheet rev D is wrong, the page size is 64w/128b
	"atmega8u2 avr:1e9389 flash=8192:128,eeprom=256:4"
		" lb:0,1 blb0:2,3 blb1:4,5"
		" cksel:8,9,10,11 sut:12,13 ckout:14 ckdiv8:15"
		" bootrst:16 bootsz:17,18 eesave:19 wdton:20 spien:21:0 rstdsbl:22:1 dwen:23:1"
		" bodlevel:24,25,26 hwbe:27\n"

	"atxmega128a avrx:1e974600 flash=139264:512,eeprom=2048:32 jtaguid:0,1,2,3,4,5,6,7 wdper:8,9,10,11 wdwper:12,13,14,15 bodpd:16,17 bootrst:22 jtagen:32 wdlock:33 startuptime:34,35 rstdisbl:36:1"
		" bodlevel:40,41,42 eesave:43 bodact:44,45 lb:56,57 blbat:58,59 blba:60,61 blbb:62,63\n"
	"atxmega64a avrx:1e964600 flash=69632:256,eeprom=2048:32 jtaguid:0,1,2,3,4,5,6,7 wdper:8,9,10,11 wdwper:12,13,14,15 bodpd:16,17 bootrst:22 jtagen:32 wdlock:33 startuptime:34,35 rstdisbl:36:1"
		" bodlevel:40,41,42 eesave:43 bodact:44,45 lb:56,57 blbat:58,59 blba:60,61 blbb:62,63\n"
	"atxmega32a avrx:1e954100 flash=36864:256,eeprom=1024:32 jtaguid:0,1,2,3,4,5,6,7 wdper:8,9,10,11 wdwper:12,13,14,15 bodpd:16,17 bootrst:22 jtagen:32 wdlock:33 startuptime:34,35 rstdisbl:36:1"
		" bodlevel:40,41,42 eesave:43 bodact:44,45 lb:56,57 blbat:58,59 blba:60,61 blbb:62,63\n"
	"atxmega16a avrx:1e944100 flash=20480:256,eeprom=1024:32 jtaguid:0,1,2,3,4,5,6,7 wdper:8,9,10,11 wdwper:12,13,14,15 bodpd:16,17 bootrst:22 jtagen:32 wdlock:33 startuptime:34,35 rstdisbl:36:1"
		" bodlevel:40,41,42 eesave:43 bodact:44,45 lb:56,57 blbat:58,59 blba:60,61 blbb:62,63\n"
	;

void update_chipdef(std::vector<chipdef> const & templates, chipdef & cd)
{
	for (std::size_t i = 0; i < templates.size(); ++i)
	{
		chipdef const & templ = templates[i];
		if (cd.signature == templ.signature)
		{
			cd.name = templ.name;
			cd.memories.insert(templ.memories.begin(), templ.memories.end());

			for (std::size_t j = 0; j < templ.fuses.size(); ++j)
			{
				std::size_t k;
				for (k = 0; k < cd.fuses.size(); ++k)
				{
					if (cd.fuses[k].name == templ.fuses[j].name)
						break;
				}

				if (k == cd.fuses.size())
					cd.fuses.push_back(templ.fuses[j]);
			}
		}
	}

	if (cd.memories.find("fuses") == cd.memories.end())
	{
		if (cd.signature.substr(0, 4) == "avr:")
		{
			chipdef::memorydef mem;
			mem.memid = 3;
			mem.size = 4;
			mem.pagesize = 0;
			cd.memories["fuses"] = mem;
		}
		else if (cd.signature.substr(0, 5) == "avrx:")
		{
			chipdef::memorydef mem;
			mem.memid = 3;
			mem.size = 8;
			mem.pagesize = 0;
			cd.memories["fuses"] = mem;
		}
	}
}

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
			memdef.memid = i + 1;
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
