#include "csvrow.h"

std::size_t CSVrow::size() const
{
	return m_data.size();
}

std::string const& CSVrow::operator[](std::size_t index) const
{
	return m_data[index];
}

void CSVrow::readNextRow(std::istream& str)
{
	std::string line;
	std::getline(str, line);

	std::stringstream lineStream(line);
	std::string cell;

	m_data.clear();
	while (std::getline(lineStream, cell, ','))
	{
		m_data.push_back(cell);
	}
	// This checks for a trailing comma with no data after it.
	if (!lineStream && cell.empty())
	{
		// If there was a trailing comma then add an empty element.
		m_data.push_back("");
	}
}

std::istream& CSVrow::operator<<(std::istream& str)
{
	this->readNextRow(str);
	return str;
}

// friend function

std::istream& operator>>(std::istream& str, CSVrow& data)
{
	data.readNextRow(str);
	return str;
}