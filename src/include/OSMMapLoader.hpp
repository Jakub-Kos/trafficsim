#pragma once
#include <string>

class OSMMapLoader {
public:
    /**
     * @brief Parses OSM XML data and converts it to the simulation's internal JSON map format.
     * @param osmXmlData The raw string content of the .osm file.
     * @return std::string A JSON string representing the map (nodes, ways, lanes).
     */
    static std::string parseOSMtoJSON(const std::string& osmXmlData);
};