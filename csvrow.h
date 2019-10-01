#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <sstream>

class CSVrow
{
public:
	std::string const& operator[](std::size_t index) const;
	std::size_t size() const;
	void readNextRow(std::istream& str);
	std::istream& operator<<(std::istream& str);
private:
	std::vector<std::string> m_data;
};

std::istream& operator>>(std::istream& str, CSVrow& data);

