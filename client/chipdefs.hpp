#ifndef AVRICSP_CLIENT_CHIPDEFS_HPP
#define AVRICSP_CLIENT_CHIPDEFS_HPP

#include <string>
#include <vector>
#include <map>
#include <stdint.h>

extern const std::string embedded_chipdefs;

struct chipdef
{
	std::string name;
	std::string signature;
	
	struct memorydef
	{
		int size;
		int pagesize;
	};
	
	std::map<std::string, memorydef> memories;
	
	struct fuse
	{
		std::string name;
		std::vector<int> bits;
		std::vector<int> values;
	};
	
	std::vector<fuse> fuses;
	
	std::string format_value(uint8_t const * first, uint8_t const * last) const;
	bool is_value_safe(uint8_t const * first, uint8_t const * last) const;
};

void parse_chipdefs(std::string const & strdefs, std::vector<chipdef> & res);


#endif