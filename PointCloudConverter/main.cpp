// PointCloudConverter
//
// This console application takes in a text file containing 3D point cloud data
// and generates a triangulation. It outputs a text file with vertex, index and neighbor data.


#include <fstream>
#include <iomanip>
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

	Vector3 operator+(const Vector3& v) const
    {
    	return Vector3{ x + v.x, y + v.y, z + v.z };
	}

	Vector3 operator-(const Vector3& v) const
    {
    	return Vector3{ x - v.x, y - v.y, z - v.z };
	}
};

typedef std::vector<std::vector<std::vector<Vector3>>> PointCloudGrid;

/**
 * \param grid The point cloud grid
 * \param x The x index of the cell
 * \param y The y index of the cell
 * \return The average height of all points in the cell
 */
float getHeight(PointCloudGrid& grid ,int x, int y)
{
	float sum = 0.f;

	try
	{
		for (auto v : grid[x][y])
			sum += v.z;
	}
	catch(...)
	{
	}

    return sum / grid[x][y].size();
}

std::vector<Vector3> readVertexData(const std::string& fileName)
{
	std::vector<Vector3> vertexDataRaw;

	std::ifstream file(fileName);

	if (!file.is_open())
		throw std::runtime_error("Could not open given file.");

	Vector3 v;
	while (file >> v.x >> v.y >> v.z)
		vertexDataRaw.emplace_back(v);

    file.close();

	return vertexDataRaw;
}

Bounds findBounds(const std::vector<Vector3>& vertexData)
{
    Bounds b;

	b.xmin = b.xmax = vertexData[0].x;
	b.ymin = b.ymax = vertexData[0].y;

    for (auto v : vertexData)
    {
        if (v.x > b.xmax) b.xmax = v.x;
        if (v.x < b.xmin) b.xmin = v.x;
        if (v.y > b.ymax) b.ymax = v.y;
        if (v.y < b.ymin) b.ymin = v.y;
    }

    b.xSize = b.xmax - b.xmin;
    b.ySize = b.ymax - b.ymin;

    return b;
}

void mergeVertices(PointCloudGrid& grid)
{
	for (int x = 0; x < grid.size(); ++x)
	{
		for (int y = 0; y < grid[x].size(); ++y)
		{
			if (grid[x][y].size() > 1)
			{
				Vector3 v;
				for (auto p : grid[x][y])
				{
					v.x += p.x;
					v.y += p.y;
					v.z += p.z;
				}

				v.x /= grid[x][y].size();
				v.y /= grid[x][y].size();
				v.z /= grid[x][y].size();

				grid[x][y].clear();
				grid[x][y].emplace_back(v);
			}
		}
	}
}

int main(int argc, char* argv[])
{

    std::string fileName = "vertexData.txt";
    std::vector<Vector3> vertexDataRaw = readVertexData(fileName);

	/*std::ofstream outFile("vertexDataRaw.txt");
	if (!outFile.is_open()) return 1;

	for (int i = 0; i < vertexDataRaw.size(); i += 1000)
		outFile << vertexDataRaw[i].x << " " << vertexDataRaw[i].y << " " << vertexDataRaw[i].z << "\n";

	return 0;*/

	/*for (int i = 0; i < vertexDataRaw.size(); i+=10000)
		std::cout << vertexDataRaw[i].x << " " << vertexDataRaw[i].y << " " << vertexDataRaw[i].z << std::endl;*/

	const Vector3 offset = vertexDataRaw[0];

	// Remove pivot
	for (auto v : vertexDataRaw)
	{
		v = v - offset;
	}


    Bounds bounds = findBounds(vertexDataRaw);
	std::cout << std::fixed << std::setprecision(2) << "xmin " << bounds.xmin << " xmax " << bounds.xmax << " ymin " << bounds.ymin << " ymax " << bounds.ymax << std::endl;
	std::cout << std::fixed << std::setprecision(2) << "xSize " << bounds.xSize << " ySize " << bounds.ySize << std::endl;

	return 0;
	

	// Distance between each cell wall
	int numSteps = 10;
	/*int numStepsX = ceil(bounds.xSize / stepLength);
	int numStepsY = ceil(bounds.ySize / stepLength);*/

	//std::cout << numStepsX << " " << numStepsY << std::endl;

	//// Rows and columns, where each cell is a vector of points in that area. Initialize empty cells.
	PointCloudGrid pointCloudGrid(numSteps, std::vector<std::vector<Vector3>>(numSteps, std::vector<Vector3>()));

	// The final grid containing one vertex per cell.
	std::vector<std::vector<Vector3>> vertexGrid(numSteps, std::vector<Vector3>(numSteps, Vector3()));

	//---------------
	//|  x| x |
	//|   |x x|
	//---------------
	//|x  |   |
	//|  x|x  |

    // Put points into vertexList
    for (auto v : vertexDataRaw)
    {
		std::cout << v.x << " - " << bounds.xmin << " / " << bounds.xSize << std::endl;
        int x = static_cast<int>(ceil(v.x / bounds.xSize));
        int y = static_cast<int>(ceil(v.y / bounds.ySize));

		std::cout << x << std::endl;
        //pointCloudGrid[x][y].emplace_back(v);
    }



    
    /*for (float y = bounds.ymin; y < bounds.ymax; y += stepLength)
    {
		std::vector<std::vector<Vector3>> row;
        for (float x = bounds.xmin; x < bounds.xmax; x += stepLength)
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
	}*/



    return 0;
}
