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
		int memid;
		size_t size;
		size_t pagesize;
	};
	
	std::map<std::string, memorydef> memories;
	
	struct fuse
	{
		std::string name;
		std::vector<int> bits;
		std::vector<int> values;
	};
	
	std::vector<fuse> fuses;
	
	template <typename Iter>
	std::string format_value(Iter first, Iter last) const
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

	template <typename Iter>
	bool is_value_safe(Iter first, Iter last) const
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
};

void parse_chipdefs(std::string const & strdefs, std::vector<chipdef> & res);
void update_chipdef(std::vector<chipdef> const & templates, chipdef & cd);


#endif