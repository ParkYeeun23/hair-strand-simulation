#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "Vec3.h"

using namespace std;

class Vertex2
{
public:
	Vec3<float> position;
	Vec3<float> normal;
	vector<int> neighborFaces;
};

class Face
{
public:
	int vertexIdx[3];
	Vec3<float> v0, v1, v2;
	Vec3<float> normal;
};

class modelLoad
{
public:
	vector<Vertex2> vertexArray;
	vector<Face> faceArray;

	// spatial grid for triangle collision queries (cell -> triangle indices)
	Vec3<float> m_GridMin;
	Vec3<float> m_GridMax;
	float m_CellSize = 1.0f;
	int m_DimX = 0, m_DimY = 0, m_DimZ = 0;
	vector<vector<int>> m_Cells;

	bool LoadMeshfile(string filename);
	void ComputeFaceNormal();
	void FindNeighborFaces();
	void ComputeVertexNormal();

	void BuildCollisionGrid(float cellSize);
	bool ResolveCollision(Vec3<float>& p, float offset);
	Vec3<float> Center();

public:
	void RenderMesh();

private:
	int CellIndex(int ix, int iy, int iz) { return ix + iy * m_DimX + iz * m_DimX * m_DimY; }
};