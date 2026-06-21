#pragma once
#include "Vec3.h"
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
using namespace std;

// fallback sphere collider used when there is no mesh
const Vec3<float> HEAD_CENTER(0.0f, -1.0f, 0.0f);
const float       HEAD_RADIUS = 5.0f;

class modelLoad;

class hairstrand
{
public:
	float m_Length;
	int m_Count;
	Vec3<float> m_InitPos;
	Vec3<float> m_InitDir = Vec3<float>(0.0f, 1.0f, 0.0f);	// growth direction from the root
	Vec3<float> m_HeadCenter;	// orients the hair ribbon
	vector<Vec3<float>> m_Pos;
	vector<Vec3<float>> m_NewPos;
	vector<Vec3<float>> m_Vel;
	vector<float> m_Mass;
	vector<float> m_InvMass;	// 0 = pinned
	vector<float> m_DihedralAngle;
	modelLoad* m_Collider = nullptr;	// mesh collider, or null for the sphere
	float m_CollisionRadius = 0.15f;	// standoff from the surface
public:
	float m_Strt_RestLength;
	float m_Bend_RestLength;
public:
	hairstrand();
	hairstrand(float length, int count);
	hairstrand(Vec3<float> position, float length, int count);
	~hairstrand();
public:
	inline Vec3<float> getPos(int i) { return m_Pos[i]; }
	inline void SetCollider(modelLoad* m) { m_Collider = m; }
	inline void SetInitDir(Vec3<float> dir) { m_InitDir = dir; }
	inline void SetHeadCenter(Vec3<float> c) { m_HeadCenter = c; }
	void Init();
	void Simulation(float dt);
	void Integrate(float dt);
	void ComputeRestLength();
	void UpdateStructuralSprings();
	void UpdateBendSprings();
	void SolveDistanceConstraint(int index0, int index1, float restLength);
	void SolveCollisionConstraint(Vec3<float> center, float radius);
	void SolveMeshCollisionConstraint(float offset);
	void ApplyExternalForces(float dt);
	void DrawLine();
	void Draw();
	void ApplyWind(Vec3<float> wind);
	void ApplyWindForRope(Vec3<float> wind, int index0, int index1);
};