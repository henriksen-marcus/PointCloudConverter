// PointCloudConverter
//
// https://github.com/henriksen-marcus/PointCloudConverter
//
// This console application generates a triangulation based on a text file containing 3D point cloud data.
// It groups the points into cells, and generates a common vertex for each cell. It then generates indices for
// the triangles. It outputs a text file with vertex data, and a text file with index and neighbor data.


#include <chrono>
#include <fstream>
#include <iomanip>
#include <string>
#include <iostream>
#include <vector>
#include "Timer.h"


static constexpr int DECIMAL_PRECISION = 4;
#define SET_PRECISION std::fixed << std::setprecision(DECIMAL_PRECISION)

// Distance between each cell wall. Configure this to change the output mesh resolution.
static constexpr float STEP_LENGTH = 5.f;

// Flip the y and z coordinates on export. Useful when exporting to e.g. Unity. 
static constexpr bool FLIP_YZ = true;


struct Bounds
{
	float xmin{}, ymin{}, zmin{}, xmax{}, ymax{}, zmax{}, xSize{}, ySize{}, zSize{};
};

struct Vector3
{
    float x{}, y{}, z{};
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

	file << vertexData.size() * vertexData[0].size() << "\n";

	for (const auto& i : vertexData)
		for (const auto& j : i)
			if (FLIP_YZ)
				file << SET_PRECISION << j.x << " " << j.z << " " << j.y << "\n";
			else
				file << SET_PRECISION << j.x << " " << j.y << " " << j.z << "\n";

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
	if (vertexData.empty())
		throw std::runtime_error("Input vector is empty");

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
			//std::cout << "Cell empty.\n";
			return lastHeight;
		}

		// Calculate the average height of all vertices in the cell.
		for (const auto& v : cell)
			zSum += v.z;

		float height = zSum /= cell.size();

		return height;
	}
	catch (std::out_of_range&) {}

	return zSum;
}

/**
 * \brief Merge vertices in each cell into one vertex placed in the top left, with the average height.
 * \param grid The point cloud grid.
 * \param stepLength The distance between each cell wall.
 * \return A grid containing one vertex per cell.
 */
std::vector<std::vector<Vector3>> mergeVertices(const PointCloudGrid& grid, float stepLength)
{
	// Initialize a grid containing one vertex per cell.
	std::vector<std::vector<Vector3>> vertexGrid(grid.size(), std::vector<Vector3>(grid[0].size(), Vector3()));

	for (int x = 0; x < grid.size(); x++)
	{
		for (int y = 0; y < grid[0].size(); y++)
		{
			const float height = getHeight(grid, x, y);
			lastHeight = height;

			vertexGrid[x][y] = { x * stepLength, y * stepLength, height };
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

	Timer timer;
	timer.Start();

	while (std::getline(in, line))
	{
		out << line << "\n";

		for (int i = 0; i < skipLines; i++)
			std::getline(in, line);
	}

	std::cout << "End thinning data. Time taken: " << timer.Stop() << " seconds\n";
}

/**
 * \brief Generates triangle index information and neighbor
 * information for the given vertex grid.
 * \param vertexData A grid of vertices.
 * \return A compiled list of indices and neighbors for each triangle.
 * Six lines per triangle, where the first three lines are the indices.
 */
std::vector<int> generateIndices(const std::vector<std::vector<Vector3>>& vertexData)
{
	std::vector<std::vector<int>> indices;
	std::vector<std::vector<int>> neighbors;

	// Number of vertices in the x direction.
	int n_x = vertexData.size();

	// Number of vertices in the y direction.
	int n_y = vertexData[0].size();

	Timer t;
	t.Start();

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
			std::vector<int> triangle;
			triangle.emplace_back(top_left);
			triangle.emplace_back(bottom_left);
			triangle.emplace_back(bottom_right);

			indices.emplace_back(triangle);

			// Triangle 1
			triangle.clear();
			triangle.emplace_back(top_left);
			triangle.emplace_back(bottom_right);
			triangle.emplace_back(top_right);

			indices.emplace_back(triangle);

			continue;
			//std::cout << "top_left: " << top_left << "\nbottom_left: " << bottom_left << "\nbuttom_right: " << buttom_right << "\ntop_right: " << top_right << std::endl;

			/*
			 * Generating neighbor data: Overview of the indices for each triangle in the grid.
			 *
			 * T is our current triangle.
			 *
			 * To generate neighbor information, we take the opposite triangle
			 * index of the vertex that we are on. For example, the first neighbor
			 * for triangle T is T0, which is on the opposite side of the first vertex
			 * (marked with an x) which is on the top left of the square.
			 *
			 * The second neighbor is T1, which is on the opposite side of the second vertex.
			 * Finally the third neighbor is T2, which is on the opposite side of the third vertex.
			 *
			 * The neighbor data should then be formatted like this: T0 T1 T2
			 * Where the T's represent the index of the triangle in the indices array.
			 *
			 * If a neighbor doesn't exist (out of bounds in indices array), we set it to -1.
			 *
			 * -|---|---|---|-
			 *  |\  |\  |\  |
			 *  | \ | \ | \ |
			 *  |  \|T4\|  \|
			 * -|---x---|---|-
			 *  |\T2|\T1|\  |
			 *  | \ | \ | \ |
			 *  |  \|T \|T3\|
			 * -|---|---|---|-
			 *  |\  |\T0|\  |
			 *  | \ | \ | \ |
			 *  |  \|  \|  \|
			 * -|---|---|---|-
			 */

			constexpr int one_step_down = 2;
			const int one_step_right = 2 * (n_y - 1);

			// Current triangle index (in the indices array)
			int T = 2 * (j + i * (n_y - 1));

			int T0 = T + one_step_down + 1; // Second triangle in next square
			int T1 = T + 1; // Next triangle
			int T2 = T - one_step_right + 1; // Second triangle in previous column the our left
			int T3 = T + one_step_right; // First triangle in next column to our right
			int T4 = T - one_step_down;

			neighbors.emplace_back();
			auto& tri_neighbors = neighbors.back();

			// Add neighbor data for the first triangle

			// Check if the neighbor exists

			/* If the j vertex we are on is more than or equal to the one before the last, then there won't be a T0 triangle.
			 * Similarly, if our i is 0, there won't be a T2 triangle. */

			/* If our y index is equal or more than the number of vertices
			 * in the y direction minus one, there won't be a T0 triangle. */
			tri_neighbors.emplace_back(j < (n_y - 1) ? T0 : -1);

			tri_neighbors.emplace_back(T1);

			// If our x index is 0, there won't be a T2 triangle.
			tri_neighbors.emplace_back(i > 0 ? T2 : -1);


			neighbors.emplace_back();
			tri_neighbors = neighbors.back();

			// If our x index is equal or more than the number of vertices in the x direction minus one, there won't be a T3 triangle.
			tri_neighbors.emplace_back(i < (n_x - 1) ? T3 : -1);

			// If our y index is 0, there won't be a T4 triangle.
			tri_neighbors.emplace_back(j > 0 ? T4 : -1);

			tri_neighbors.emplace_back(T);
		}
	}

	t.Println();

	std::vector<int> merged;
	/*
	for (int i = 0; i < indices.size(); i++)
	{
		merged.insert(merged.end(), indices[i].begin(), indices[i].end());
		merged.insert(merged.end(), neighbors[i].begin(), neighbors[i].end());
	}	

	return merged;*/

	for (const auto& tri : indices)
	{
		merged.insert(merged.end(), tri.begin(), tri.end());
	}

	return merged;
}

int main()
{
	std::cout << "Point Cloud Converter\n";

    std::string fileName = "sampleData.txt";
	std::vector<Vector3> vertexDataRaw = readVertexData(fileName);

	//thinData(fileName, "thinnedVertexData.txt", 500);
	//return 0;

	Bounds bounds = findBounds(vertexDataRaw);
	//const Vector3 offset = vertexDataRaw[0];

	// Remove world offset
	for (auto& v : vertexDataRaw)
	{
		v.x -= bounds.xmin;
		v.y -= bounds.ymin;
		v.z -= bounds.zmin;
	}

	bounds = findBounds(vertexDataRaw);

	std::cout << "Data bounds:\n";
	std::cout << SET_PRECISION << "xmin: " << bounds.xmin << " xmax: " << bounds.xmax << "\nymin: " << bounds.ymin << " ymax: " << bounds.ymax << "\n";
	std::cout << SET_PRECISION << "xSize: " << bounds.xSize << " ySize: " << bounds.ySize << "\n\n";

	// Number of cells in each direction. Determines output vertex resolution.
	const int numCellsX = static_cast<int>(ceil(bounds.xSize / STEP_LENGTH)+1);
	const int numCellsY = static_cast<int>(ceil(bounds.ySize / STEP_LENGTH)+1);

	// Rows and columns, where each cell is a vector of points in that area. Initialize empty cells.
	PointCloudGrid pointCloudGrid(numCellsX, std::vector<std::vector<Vector3>>(numCellsY, std::vector<Vector3>()));

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
		//std::cout << "y " << v.y << " - " << bounds.ymin << " / " << stepLength << std::endl;

        int x = static_cast<int>(floor(v.x / STEP_LENGTH));
        int y = static_cast<int>(floor(v.y / STEP_LENGTH));

		// When reaching the point exactly on the edge of the bounds, the index will be out of range.
		x = std::min(x, numCellsX - 1);
		y = std::min(y, numCellsY - 1);

		//std::cout << "x: " << v.x << " Steplength: " << stepLength << " xSize: " << bounds.xSize << " Result: " << x << "\n";
		//std::cout << "x: " << x << " y: " << y << std::endl;

		pointCloudGrid[x][y].emplace_back(v);
    }

	const auto vertexGrid = mergeVertices(pointCloudGrid, STEP_LENGTH);
	exportVertexData(vertexGrid, "newVertexData.txt");

	const auto indices = generateIndices(vertexGrid);
	exportIndexData(indices, "newIndexData.txt");

	std::cout << "Output grid size: " << vertexGrid.size() << "x" << vertexGrid[0].size() << "\n";
	std::cout << "Number of vertices: " << vertexGrid.size() * vertexGrid[0].size() << "\n";
	std::cout << "Number of triangles: " << indices.size() / 3 << "\n\n";

    return 0;
}
