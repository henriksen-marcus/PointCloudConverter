// PointCloudConverter
//
// This console application generates a triangulation based on a text file containing 3D point cloud data.
// It groups the points into cells, and generates a vertex for each cell. It then generates indices for the
// triangles. It outputs a text file with vertex, and a text file with index and neighbor data.


#include <chrono>
#include <fstream>
#include <iomanip>
#include <string>
#include <iostream>
#include <sstream>
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

/* In case of a cell not containing any vertices, we apply the
 * height of the last cell to avoid extreme height changes. */
float lastHeight{};

/**
 * \brief A 2D grid of cells, where each cell contains a list (vector) of vertices.
 */
typedef std::vector<std::vector<std::vector<Vector3>>> PointCloudGrid;

std::vector<Vector3> readVertexData(const std::string& fileName)
{
	std::cout << "Begin read vertex data" << std::endl;

	std::vector<Vector3> vertexData;

	std::ifstream file(fileName);

	if (!file.is_open())
		throw std::runtime_error("Could not open given file.");

	Vector3 v;
	while (file >> v.x >> v.y >> v.z)
		vertexData.emplace_back(v);

    file.close();

	std::cout << "End read vertex data" << std::endl;

	return vertexData;
}

void exportVertexData(const std::vector<std::vector<Vector3>>& vertexData, const std::string& fileName)
{
	std::ofstream file(fileName);

	if (!file.is_open())
		throw std::runtime_error("Could not open the created file.");

	file << vertexData.size() * vertexData.size() << "\n";

	for (const auto& i : vertexData)
		for (const auto& j : i)
			file << SET_PRECISION << j.x << " " << j.z << " " << j.y << "\n";

	file.close();
}

void exportIndexData(const std::vector<int>& indexData, const std::string& fileName)
{
	std::ofstream file(fileName);

	if (!file.is_open())
		throw std::runtime_error("Could not open the created file.");

	file << indexData.size() / 3 << "\n";

	for (int i = 0; i < indexData.size(); i+= 3)
		file << indexData[i] << " " << indexData[i+1] << " " << indexData[i+2] <<"\n";

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
 * \param grid The point cloud grid
 * \param x The x index of the cell
 * \param y The y index of the cell
 * \return The average height of all points in the cell
 */
float getHeight(const PointCloudGrid& grid, int x, int y)
{
	float zSum{};

	try
	{
		const auto& cell = grid[x][y];
		if (cell.empty())
		{
			std::cout << "Cell empty.\n";
			return lastHeight;
		}

		// Calculate the average height of all vertices in the cell.
		for (const auto& v : cell)
			zSum += v.z;

		float height = zSum /= cell.size();
		lastHeight = height;

		return height;
	}
	catch (std::out_of_range&) {}

	return zSum;
}

/**
 * \brief Merge vertices in each cell into one vertex placed in the top left, with the average height.
 * \param grid The point cloud grid.
 * \return A grid containing one vertex per cell.
 */
std::vector<std::vector<Vector3>> mergeVertices(const PointCloudGrid& grid, float stepLengthX, float stepLengthY)
{
	// Initialize a grid containing one vertex per cell.
	std::vector<std::vector<Vector3>> vertexGrid(grid.size(), std::vector<Vector3>(grid.size(), Vector3()));

	for (int x = 0; x < grid.size(); x++)
	{
		for (int y = 0; y < grid[0].size(); y++)
		{
			vertexGrid[x][y] = { x * stepLengthX, y * stepLengthY, getHeight(grid, x, y) };
		}
	}

	return vertexGrid;
}

/**
 * \brief Thin the data in the given file by skipping a given number of lines between each vertex.
 * \param inFile The input file name.
 * \param outFile The output file name.
 * \param skipLines Number of lines to skip between each vertex.
 */
void thinData(const std::string& inFile, const std::string& outFile, int skipLines)
{
	std::ifstream in(inFile);
	if (!in.is_open())
		throw std::runtime_error("Could not open the input file.");

	std::ofstream out(outFile);
	if (!out.is_open()) 
		throw std::runtime_error("Could not open the created file.");

	std::string line;

	std::cout << "Begin thinning data\n";

	auto start = std::chrono::high_resolution_clock::now();

	while (std::getline(in, line))
	{
		out << line << "\n";

		for (int i = 0; i < skipLines; i++)
			std::getline(in, line);
	}

	auto end = std::chrono::high_resolution_clock::now(); // Record end time
	std::chrono::duration<double> duration = end - start;

	std::cout << "End thinning data\n";
	std::cout << "Time taken: " << duration.count() << " seconds" << std::endl;
}

std::vector<int> generateIndices(const std::vector<std::vector<Vector3>>& vertexData)
{
	std::vector<int> indices;

	// Number of vertices in the x direction.
	int n_x = vertexData.size();

	// Number of vertices in the y direction.
	int n_y = vertexData[0].size();

	// X
	for (int i = 0; i < vertexData.size(); i++)
	{
		// Y
		for (int j = 0; j < vertexData[0].size(); j++)
		{
			/* The plan is to add six indices for each loop.
			 *
			 * x---x
			 * |\ 1|   The numbers here represent the
			 * | \ |   triangle number.
			 * |0 \|
			 * x---x
			 * 
			 * We start at the top left corner of the square, and add
			 * the indices for the vertices below and to the right.
			 *
			 * 0
			 * |\      The numbers here represent the vertex order
			 * | \     when adding indices to the array. We employ
			 * |  \    an anti-clockwise order to get correct normals.
			 * 1---2  
			 */

			// Skip the last row and column, since there aren't additional vertices beyond them.
			if (i == n_x - 1 || j == n_y - 1) continue;

			// Current index
			int top_left = i * n_y + j;
			int bottom_left = top_left + 1;
			int bottom_right = bottom_left + n_y;
			int top_right = top_left + n_y;

			/*
			 * 0---3---6
			 * |   |   |
			 * |   |   |   Vertices in the grid are ordered like this.
			 * 1---4---7   Therefore, to go on step to the right, we add
			 * |   |   |   the number of vertices in the y direction (n_y).
			 * |   |   |
			 * 2---5---8
			 */

			// Triangle 0
			indices.emplace_back(top_left);
			indices.emplace_back(bottom_left);
			indices.emplace_back(bottom_right);

			// Triangle 1
			indices.emplace_back(top_left);
			indices.emplace_back(bottom_right);
			indices.emplace_back(top_right);

			//std::cout << "top_left: " << top_left << "\nbottom_left: " << bottom_left << "\nbuttom_right: " << buttom_right << "\ntop_right: " << top_right << std::endl;
		}
	}

	return indices;
}

int main()
{
	std::cout << "Point Cloud Converter\n";

    std::string fileName = "thinnedVertexData.txt";
	std::vector<Vector3> vertexDataRaw = readVertexData(fileName);

	//thinData(fileName, "thinnedVertexData.txt", 1000);
	//return 0;

	/*for (int i = 0; i < vertexDataRaw.size(); i+=10000)
		std::cout << vertexDataRaw[i].x << " " << vertexDataRaw[i].y << " " << vertexDataRaw[i].z << std::endl;*/

	const Vector3 offset = vertexDataRaw[0];

	// Remove world offset
	for (auto& v : vertexDataRaw)
		v = v - offset;

	Bounds bounds = findBounds(vertexDataRaw);

	std::cout << "Data bounds:\n";
	std::cout << SET_PRECISION << "xmin: " << bounds.xmin << " xmax: " << bounds.xmax << "\nymin: " << bounds.ymin << " ymax: " << bounds.ymax << "\n";
	std::cout << SET_PRECISION << "xSize: " << bounds.xSize << " ySize: " << bounds.ySize << "\n";

	// Number of cells in each direction. Determines output vertex resolution.
	const int numCells = 20;

	/* Distance between each cell wall.
	 * Calculate the step length so that the number of steps is equal in
	 * both directions, and that it fits perfectly into the bounds. */
	float stepLengthX = bounds.xSize / (numCells - 1);
	float stepLengthY = bounds.ySize / (numCells - 1);

	// Rows and columns, where each cell is a vector of points in that area. Initialize empty cells.
	PointCloudGrid pointCloudGrid(numCells, std::vector<std::vector<Vector3>>(numCells, std::vector<Vector3>()));

	/*  -----------
	 * |x  |x  |   |  Group points into cells like this.
	 * |x  |  x|  x|  Each cell is an std::vector of points.
	 *  -----------   Each cell is converted into a vertex,
	 * |x x|   |  x|  with the position being the top left corner
	 * |x x|   |x  |  of the cell, and the average height of all the points.
	 */

    // Put points into the correct cell.
    for (const auto& v : vertexDataRaw)
    {
        int x = static_cast<int>((v.x - bounds.xmin) / stepLengthX);
        int y = static_cast<int>((v.y - bounds.ymin) / stepLengthY);

        pointCloudGrid[x][y].emplace_back(v);
    }

	auto vertexGrid = mergeVertices(pointCloudGrid, stepLengthX, stepLengthY);

	std::cout << "Output grid size: " << vertexGrid.size() << "x" << vertexGrid[0].size() << std::endl;

	exportVertexData(vertexGrid, "newVertexData.txt");

	auto indices = generateIndices(vertexGrid);
	exportIndexData(indices, "newIndexData.txt");

    return 0;
}
