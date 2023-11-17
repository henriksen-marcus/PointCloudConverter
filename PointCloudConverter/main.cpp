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

#define DECIMAL_PRECISION 4
#define SET_PRECISION std::fixed << std::setprecision(DECIMAL_PRECISION)

struct Bounds
{
	float xmin{}, ymin{}, zmin{}, xmax{}, ymax{}, zmax{}, xSize{}, ySize{}, zSize{};
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


/**
 * \brief A 2D grid of cells, where each cell contains a list (vector) of vertices.
 */
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
	std::cout << "Begin read vertex data" << std::endl;

	std::vector<Vector3> vertexDataRaw;

	std::ifstream file(fileName);

	if (!file.is_open())
		throw std::runtime_error("Could not open given file.");

	Vector3 v;
	while (file >> v.x >> v.y >> v.z)
		vertexDataRaw.emplace_back(v);

    file.close();

	std::cout << "End read vertex data" << std::endl;

	return vertexDataRaw;
}

void exportVertexData(const std::vector<std::vector<Vector3>>& vertexData, const std::string& fileName)
{
	std::ofstream file(fileName);

	if (!file.is_open())
		throw std::runtime_error("Could not open the created file.");

	file << vertexData.size() * vertexData.size() << "\n";

	for (const auto& i : vertexData)
		for (const auto& j : i)
			file << SET_PRECISION << j.x << " " << j.y << " " << j.z << "\n";

	file.close();
}

/**
 * \brief Calculate the min and max values of the given vertex data, and the size of the bounds.
 * \param vertexData Raw vertex data.
 * \return Bounds object for the vertex data.
 */
Bounds findBounds(const std::vector<Vector3>& vertexData)
{
    Bounds b;

	b.xmin = b.xmax = vertexData[0].x;
	b.ymin = b.ymax = vertexData[0].y;
	b.zmin = b.zmax = vertexData[0].z;

    for (const auto& v : vertexData)
    {
        if (v.x > b.xmax) b.xmax = v.x;
        if (v.x < b.xmin) b.xmin = v.x;
        if (v.y > b.ymax) b.ymax = v.y;
        if (v.y < b.ymin) b.ymin = v.y;
		if (v.z > b.zmax) b.zmax = v.z;
		if (v.z < b.zmin) b.zmin = v.z;
    }

    b.xSize = b.xmax - b.xmin;
    b.ySize = b.ymax - b.ymin;
	b.zSize = b.zmax - b.zmin;

    return b;
}

/**
 * \brief Merge vertices in each cell into one vertex placed in the top left, with the average height.
 * \param grid The point cloud grid.
 * \return A grid containing one vertex per cell.
 */
std::vector<std::vector<Vector3>> mergeVertices(PointCloudGrid& grid, float stepLengthX, float stepLengthY)
{
	// Initialize a grid containing one vertex per cell.
	std::vector<std::vector<Vector3>> vertexGrid(grid.size(), std::vector<Vector3>(grid.size(), Vector3()));

	for (int x = 0; x < grid.size(); x++)
	{
		for (int y = 0; y < grid[0].size(); y++)
		{
			const auto& cell = grid[x][y];

			if (cell.empty()) continue;

			// Calculate the average height of all vertices in the cell.
			float zSum{};

			for (const auto& v : cell)
				zSum += v.z;

			zSum /= cell.size();

			const Vector3 vertex{x * stepLengthX, y * stepLengthY, zSum };
			vertexGrid[x][y] = vertex;
		}
	}

	return vertexGrid;
}

/**
 * \brief Write a thinned set of vertices to a file.
 * \param vertexData Raw vertex data.
 * \param outFile The output file name.
 * \param skipLines Number of lines to skip between each vertex.
 */
void thinData(const std::vector<Vector3>& vertexData, const std::string& outFile, int skipLines)
{
	std::ofstream out(outFile);

	if (!out.is_open()) 
		throw std::runtime_error("Could not open the created file.");

	for (int i = 0; i < vertexData.size(); i += skipLines)
		out << SET_PRECISION << vertexData[i].x << " " << vertexData[i].y << " " << vertexData[i].z << "\n";
}


int main(int argc, char* argv[])
{
    std::string fileName = "vertexData.txt";
    std::vector<Vector3> vertexDataRaw = readVertexData(fileName);

	/*for (int i = 0; i < vertexDataRaw.size(); i+=10000)
		std::cout << vertexDataRaw[i].x << " " << vertexDataRaw[i].y << " " << vertexDataRaw[i].z << std::endl;*/

    //Bounds bounds = findBounds(vertexDataRaw);

	const Vector3 offset = vertexDataRaw[0];

	// Remove world offset
	for (auto& v : vertexDataRaw)
		v = v - offset;

	auto bounds = findBounds(vertexDataRaw);

	std::cout << SET_PRECISION << "xmin " << bounds.xmin << " xmax " << bounds.xmax << " ymin " << bounds.ymin << " ymax " << bounds.ymax << std::endl;
	std::cout << SET_PRECISION << "xSize " << bounds.xSize << " ySize " << bounds.ySize << std::endl;

	// Number of cells in each direction.
	int numSteps = 100;

	/* Distance between each cell wall.
	 * Calculate the step length so that the number of steps is equal in both directions,
	 * and so that it fits perfectly into the bounds. */
	float stepLengthX = bounds.xSize / (numSteps - 1);
	float stepLengthY = bounds.ySize / (numSteps - 1);

	// Rows and columns, where each cell is a vector of points in that area. Initialize empty cells.
	PointCloudGrid pointCloudGrid(numSteps, std::vector<std::vector<Vector3>>(numSteps, std::vector<Vector3>()));

	//---------------
	//|  x| x |
	//|   |x x|
	//---------------
	//|x  |   |
	//|  x|x  |

    // Put points into the correct cell.
    for (auto v : vertexDataRaw)
    {
        int x = static_cast<int>(floor((v.x - bounds.xmin) / stepLengthX));
        int y = static_cast<int>(floor((v.y - bounds.ymin) / stepLengthY));

		//std::cout << "x: " << x << " y: " << y << std::endl;

        pointCloudGrid[x][y].emplace_back(v);
    }

	auto vertexGrid = mergeVertices(pointCloudGrid, stepLengthX, stepLengthY);

	std::cout << vertexGrid.size() << " " << vertexGrid[0].size() << std::endl;

	exportVertexData(vertexGrid, "newVertexData.txt");

	/*for (int i = 0; i < vertexGrid.size(); i++)
	{
		for (int j = 0; j < vertexGrid[0].size(); j++)
		{
			std::cout << vertexGrid[j][i].z;
		}
		std::cout << "\n";
	}*/

    return 0;
}
