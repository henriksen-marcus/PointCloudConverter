#include <fstream>
#include <string>
#include <iostream>
#include <list>
#include <vector>

struct Bounds
{
    float xmin{}, ymin{}, xmax{}, ymax{}, xSize{}, ySize{};
};

struct Vector3
{
    float x{}, y{}, z{};

    bool isEmpty() const
    {
		return x == 0.f && y == 0.f && z == 0.f;
	}
};

int main(int argc, char* argv[])
{
    std::vector<Vector3> vertexDataRaw;
    std::vector<std::vector<std::vector<Vector3>>> vertexList;

    std::string fileName = "vertex.txt";
    std::ifstream file(fileName);

    if (!file.is_open())
        throw std::runtime_error("Could not open given file.");

    // Find bounds

    Vector3 vertex;
    while (file >> vertex.x >> vertex.y >> vertex.z)
        vertexDataRaw.emplace_back(vertex);

    Bounds b;

    for (auto v : vertexDataRaw)
    {
        if (v.x > b.xmax) b.xmax = v.x;
        if (v.x < b.xmin) b.xmin = v.x;
        if (v.y > b.ymax) b.ymax = v.y;
        if (v.y < b.ymin) b.ymin = v.y;
    }

    b.xSize = b.xmax - b.xmin;
    b.ySize = b.ymax - b.ymin;

    // Put points into rows and columns
    for (auto v : vertexDataRaw)
    {
        int x = (v.x - b.xmin) / b.xSize;
        int y = (v.y - b.ymin) / b.ySize;

        vertexList[x][y].emplace_back(v);
    }

    for (int i = 0; i < vertexList.size(); i++)
    {
        for (int j = 0; j < vertexList.size(); j++)
        {
            Vector3 sum;
            
            for (auto vertex : vertexList[i][j])
            {
				sum.x += vertex.x;
				sum.y += vertex.y;
				sum.z += vertex.z;
			}

            if (sum.isEmpty())
				continue;
		}
    }

    // How many rows and columns we want
    float stepLength = 10.f;

    
    for (float y = b.ymin; y < b.ymax; y += stepLength)
    {
		std::vector<std::vector<Vector3>> row;
        for (float x = b.xmin; x < b.xmax; x += stepLength)
        {
			std::vector<Vector3> column;
            for (auto v : vertexDataRaw)
            {
				if (v.x >= x && v.x < x + stepLength && v.y >= y && v.y < y + stepLength)
					column.emplace_back(v);
			}
			row.emplace_back(column);
		}
		vertexList.emplace_back(row);
	}

    for


    return 0;
}
