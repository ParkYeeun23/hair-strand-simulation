#include "GL\glut.h"
#include "modelLoad.h"

bool modelLoad::LoadMeshfile(string filename)
{
	//read file
	ifstream readFile(filename);
	if (!readFile.is_open())
	{
		cout << "ERROR : Failed to open file " << filename << "!" << endl;
		return false;
	}

	string line;
	char delimiter = ' ';
	int idx = 0;
	while (getline(readFile, line))
	{
		//cout << line << endl;
		string header, context;
		stringstream sline(line);
		getline(sline, header, ' ');
		if (header == "v")
		{
			//	cout << "v : header - " << header<<endl;
			Vertex2 v;
			sline >> context;
			v.position.SetX(stof(context));// cout << " next - x:" << stof(context) << endl;
			sline >> context;
			v.position.SetY(stof(context));// cout << " next - y:" << stof(context) << endl;
			sline >> context;
			v.position.SetZ(stof(context));// cout << " next - z:" << stof(context) << endl;

			vertexArray.push_back(v);
		}
		else if (header == "vt")
		{
			//	cout << "vt : header - " << header << endl;
		}
		else if (header == "vn")
		{
			/*if (vertexArray.size() != 0 && vertexArray.size() > idx)
			{
				cout << "vn : header - " << header << endl;
				Vertex v = vertexArray.at(idx++);
				sline >> context;
				v.normal.SetX(stof(context)); cout << " next - x:" << stof(context) << endl;
				sline >> context;
				v.normal.SetY(stof(context)); cout << " next - y:" << stof(context) << endl;
				sline >> context;
				v.normal.SetZ(stof(context)); cout << " next - z:" << stof(context) << endl;
			}
			*/
		}
		else if (header == "f")
		{
			//cout << "f : header - " << header << endl;

			Face f;
			sline >> context;
			f.vertexIdx[0] = stoi(context) - 1;// cout << " next - x:" << stoi(context) << endl;
			sline >> context;
			f.vertexIdx[1] = stoi(context) - 1;// cout << " next - y:" << stoi(context) << endl;
			sline >> context;
			f.vertexIdx[2] = stoi(context) - 1;// cout << " next - z:" << stoi(context) << endl;

			faceArray.push_back(f);
		}
	}

	readFile.close();
	ComputeFaceNormal();
	FindNeighborFaces();
	ComputeVertexNormal();
	return true;
}

void modelLoad::ComputeFaceNormal()
{
	for (auto& f : faceArray)
	{
		f.v0 = vertexArray[f.vertexIdx[0]].position;
		f.v1 = vertexArray[f.vertexIdx[1]].position;
		f.v2 = vertexArray[f.vertexIdx[2]].position;

		Vec3<float> va = f.v1 - f.v0;
		Vec3<float> vb = f.v2 - f.v0;
		Vec3<float> vc = va.Cross(vb);
		vc.Normalize();
		f.normal = vc;
	}
}

void modelLoad::FindNeighborFaces()
{
	for (int i = 0; i < vertexArray.size(); i++)
	{
		for (int j = 0; j < faceArray.size(); j++)
		{
			Face& f = faceArray[j];
			if (f.vertexIdx[0] == i || f.vertexIdx[1] == i || f.vertexIdx[2] == i)
			{
				vertexArray[i].neighborFaces.push_back(j);
			}
		}
	}
}

void modelLoad::ComputeVertexNormal()
{
	for (auto& v : vertexArray)
	{
		for (int i : v.neighborFaces)
		{
			v.normal += faceArray[i].normal;
		}
		v.normal /= v.neighborFaces.size();
	}
}

// closest point on triangle (a,b,c) to point p (Ericson, Real-Time Collision Detection)
static Vec3<float> ClosestPtPointTriangle(Vec3<float> p, Vec3<float> a, Vec3<float> b, Vec3<float> c)
{
	Vec3<float> ab = b - a;
	Vec3<float> ac = c - a;
	Vec3<float> ap = p - a;
	float d1 = ab.Dot(ap);
	float d2 = ac.Dot(ap);
	if (d1 <= 0.0f && d2 <= 0.0f) return a;

	Vec3<float> bp = p - b;
	float d3 = ab.Dot(bp);
	float d4 = ac.Dot(bp);
	if (d3 >= 0.0f && d4 <= d3) return b;

	float vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
	{
		float v = d1 / (d1 - d3);
		return a + ab * v;
	}

	Vec3<float> cp = p - c;
	float d5 = ab.Dot(cp);
	float d6 = ac.Dot(cp);
	if (d6 >= 0.0f && d5 <= d6) return c;

	float vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
	{
		float w = d2 / (d2 - d6);
		return a + ac * w;
	}

	float va = d3 * d6 - d5 * d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
	{
		float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		Vec3<float> bc = c - b;
		return b + bc * w;
	}

	float denom = 1.0f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	return a + ab * v + ac * w;
}

Vec3<float> modelLoad::Center()
{
	return (m_GridMin + m_GridMax) * 0.5f;
}

void modelLoad::BuildCollisionGrid(float cellSize)
{
	m_CellSize = cellSize;
	m_GridMin.Set(1e9f, 1e9f, 1e9f);
	m_GridMax.Set(-1e9f, -1e9f, -1e9f);
	for (auto& f : faceArray)
	{
		Vec3<float> verts[3] = { f.v0, f.v1, f.v2 };
		for (auto& v : verts)
			for (int k = 0; k < 3; k++)
			{
				if (v[k] < m_GridMin[k]) m_GridMin[k] = v[k];
				if (v[k] > m_GridMax[k]) m_GridMax[k] = v[k];
			}
	}
	// pad one cell so points just outside the mesh still hash in
	Vec3<float> pad(cellSize, cellSize, cellSize);
	m_GridMin -= pad;
	m_GridMax += pad;

	m_DimX = (int)ceil((m_GridMax[0] - m_GridMin[0]) / cellSize); if (m_DimX < 1) m_DimX = 1;
	m_DimY = (int)ceil((m_GridMax[1] - m_GridMin[1]) / cellSize); if (m_DimY < 1) m_DimY = 1;
	m_DimZ = (int)ceil((m_GridMax[2] - m_GridMin[2]) / cellSize); if (m_DimZ < 1) m_DimZ = 1;
	m_Cells.assign((size_t)m_DimX * m_DimY * m_DimZ, vector<int>());

	for (int t = 0; t < (int)faceArray.size(); t++)
	{
		Face& f = faceArray[t];
		Vec3<float> lo = f.v0, hi = f.v0;
		Vec3<float> rest[2] = { f.v1, f.v2 };
		for (auto& v : rest)
			for (int k = 0; k < 3; k++)
			{
				if (v[k] < lo[k]) lo[k] = v[k];
				if (v[k] > hi[k]) hi[k] = v[k];
			}
		int ix0 = (int)floor((lo[0] - m_GridMin[0]) / cellSize), ix1 = (int)floor((hi[0] - m_GridMin[0]) / cellSize);
		int iy0 = (int)floor((lo[1] - m_GridMin[1]) / cellSize), iy1 = (int)floor((hi[1] - m_GridMin[1]) / cellSize);
		int iz0 = (int)floor((lo[2] - m_GridMin[2]) / cellSize), iz1 = (int)floor((hi[2] - m_GridMin[2]) / cellSize);
		for (int iz = iz0; iz <= iz1; iz++)
			for (int iy = iy0; iy <= iy1; iy++)
				for (int ix = ix0; ix <= ix1; ix++)
				{
					if (ix < 0 || iy < 0 || iz < 0 || ix >= m_DimX || iy >= m_DimY || iz >= m_DimZ) continue;
					m_Cells[CellIndex(ix, iy, iz)].push_back(t);
				}
	}
}

// push p out to the surface if it is inside the mesh or within offset of it
bool modelLoad::ResolveCollision(Vec3<float>& p, float offset)
{
	if (m_Cells.empty()) return false;

	int cx = (int)floor((p[0] - m_GridMin[0]) / m_CellSize);
	int cy = (int)floor((p[1] - m_GridMin[1]) / m_CellSize);
	int cz = (int)floor((p[2] - m_GridMin[2]) / m_CellSize);

	int r = (int)ceil(offset / m_CellSize); if (r < 1) r = 1;

	float bestDistSq = 1e18f;
	Vec3<float> bestC, bestN;
	bool found = false;
	for (int iz = cz - r; iz <= cz + r; iz++)
		for (int iy = cy - r; iy <= cy + r; iy++)
			for (int ix = cx - r; ix <= cx + r; ix++)
			{
				if (ix < 0 || iy < 0 || iz < 0 || ix >= m_DimX || iy >= m_DimY || iz >= m_DimZ) continue;
				for (int t : m_Cells[CellIndex(ix, iy, iz)])
				{
					Face& f = faceArray[t];
					Vec3<float> cpt = ClosestPtPointTriangle(p, f.v0, f.v1, f.v2);
					Vec3<float> d = p - cpt;
					float dsq = d.LengthSquared();
					if (dsq < bestDistSq)
					{
						bestDistSq = dsq;
						bestC = cpt;
						bestN = f.normal;
						found = true;
					}
				}
			}

	if (!found) return false;

	Vec3<float> d = p - bestC;
	float dist = sqrt(bestDistSq);
	float side = d.Dot(bestN);
	if (side < 0.0f || dist < offset)
	{
		p = bestC + bestN * offset;	// snap to surface + standoff
		return true;
	}
	return false;
}

void modelLoad::RenderMesh()
{
	glColor3f(0.8f, 0.8f, 0.8f);
	for (auto& f : faceArray)
	{
		glBegin(GL_TRIANGLES);
		glNormal3f(f.normal.GetX(), f.normal.GetY(), f.normal.GetZ());
		glVertex3f(f.v0.GetX(), f.v0.GetY(), f.v0.GetZ());
		glVertex3f(f.v1.GetX(), f.v1.GetY(), f.v1.GetZ());
		glVertex3f(f.v2.GetX(), f.v2.GetY(), f.v2.GetZ());
		glEnd();
	}
}

