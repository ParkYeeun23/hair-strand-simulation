#include "hairstrand.h"
#include "modelLoad.h"
#include <GL/glut.h>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define STB_PERLIN_IMPLEMENTATION
#include <stb_perlin.h>
#include <iostream>
GLuint texture[4];
hairstrand::hairstrand()
{
}

hairstrand::hairstrand(float length, int count)
{
	m_Length = length;
	m_Count = count;
	m_Pos.resize(m_Count);
	m_NewPos.resize(m_Count);
	m_Vel.resize(m_Count);
	m_Mass.resize(m_Count);
	m_InvMass.resize(m_Count);
	m_InitPos = Vec3<float>(0.0f, 0.0f, 0.0f);

	Init();
	ComputeRestLength();
}

hairstrand::hairstrand(Vec3<float> position, float length, int count)
{
	m_Length = length;
	m_Count = count;
	m_Pos.resize(m_Count);
	m_NewPos.resize(m_Count);
	m_Vel.resize(m_Count);
	m_Mass.resize(m_Count);
	m_InvMass.resize(m_Count);
	m_InitPos = position;

	Init();
	ComputeRestLength();
}

hairstrand::~hairstrand()
{
}

void hairstrand::Init()
{
	// grow the strand from the root along m_InitDir (scalp normal, or up by default)
	Vec3<float> dir = m_InitDir;
	if (dir.LengthSquared() < 1e-8f) dir.Set(0.0f, 1.0f, 0.0f);
	dir.Normalize();

	for (int i = 0; i < m_Count; i++)
	{
		m_Mass[i] = 1.0f;
		m_InvMass[i] = (i == 0) ? 0.0f : 1.0f / m_Mass[i];	// root is pinned (invMass 0)
		m_Vel[i].Set(0.0f, 0.0f, 0.0f);
		Vec3<float> step = dir * (m_Length * i / (float)m_Count);
		m_Pos[i].Set(m_InitPos.GetX() + step.GetX(), m_InitPos.GetY() + step.GetY(), m_InitPos.GetZ() + step.GetZ());
		m_NewPos[i] = m_Pos[i];
	}
}

void hairstrand::Simulation(float dt)
{
	ApplyExternalForces(dt);

	// more iterations = stiffer springs
	int iter = 10;
	for (int i = 0; i < iter; i++)
	{
		UpdateStructuralSprings();
		UpdateBendSprings();
	}

	// collide once after the spring solve
	if (m_Collider != nullptr)
		SolveMeshCollisionConstraint(m_CollisionRadius);
	else
		SolveCollisionConstraint(HEAD_CENTER, HEAD_RADIUS);

	Integrate(dt);
}

void hairstrand::Integrate(float dt)
{
	for (int i = 1; i < m_Count; i++)
	{
		m_Vel[i] = (m_NewPos[i] - m_Pos[i]) / dt;
		m_Pos[i] = m_NewPos[i];
	}
}

void hairstrand::ComputeRestLength()
{
	m_Strt_RestLength = m_Length / (float)m_Count;
	m_Bend_RestLength = m_Strt_RestLength * 2.0f;
}

void hairstrand::UpdateStructuralSprings()
{
	for (int i = 0; i < m_Count - 1; i++)
	{
		int index0 = i;
		int index1 = i + 1;
		SolveDistanceConstraint(index0, index1, m_Strt_RestLength);
	}
}

void hairstrand::UpdateBendSprings()
{
	for (int i = 0; i < m_Count - 2; i++)
	{
		int index0 = i;
		int index1 = i + 2;
		SolveDistanceConstraint(index0, index1, m_Bend_RestLength);
	}
}

void hairstrand::SolveDistanceConstraint(int index0, int index1, float restLength)
{
	// PBD distance constraint, weighted by inverse mass
	float w0 = m_InvMass[index0];
	float w1 = m_InvMass[index1];
	float wSum = w0 + w1;
	if (wSum == 0.0f)
		return;	// both pinned

	Vec3<float> n = m_NewPos[index0] - m_NewPos[index1];
	float length = n.Length();
	if (length < 1e-8f)
		return;
	n /= length;

	float C = length - restLength;
	m_NewPos[index0] -= n * (w0 / wSum * C);
	m_NewPos[index1] += n * (w1 / wSum * C);
}

void hairstrand::SolveCollisionConstraint(Vec3<float> center, float radius)
{
	for (int i = 0; i < m_Count; i++)
	{
		if (m_InvMass[i] == 0.0f)
			continue;	// skip the root

		Vec3<float> n = m_NewPos[i] - center;
		float distance = n.Length();
		if (distance < radius && distance > 1e-8f)
		{
			// push the point out to the sphere surface
			n /= distance;
			m_NewPos[i] = center + n * radius;
		}
	}
}

void hairstrand::SolveMeshCollisionConstraint(float offset)
{
	for (int i = 0; i < m_Count; i++)
	{
		if (m_InvMass[i] == 0.0f)
			continue;	// skip the root
		m_Collider->ResolveCollision(m_NewPos[i], offset);
	}
}

void hairstrand::ApplyExternalForces(float dt)
{
	Vec3<float> gravity(0.0f, -9.8f, 0.0f);
	float damping = 0.99f;
	for (int i = 0; i < m_Count; i++)
	{
		if (m_InvMass[i] == 0.0f)
		{
			// root stays fixed
			m_Vel[i].Set(0.0f, 0.0f, 0.0f);
			m_NewPos[i] = m_Pos[i];
			continue;
		}
		m_Vel[i] += gravity * dt;
		m_Vel[i] *= damping;
		m_NewPos[i] = m_Pos[i] + m_Vel[i] * dt;
	}
}

void hairstrand::DrawLine()
{
	// thin tapered ribbon (hair card) running along the strand
	const float rootWidth = 0.16f;

	glPushAttrib(GL_ENABLE_BIT);
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);

	glBegin(GL_TRIANGLE_STRIP);
	for (int i = 0; i < m_Count; i++)
	{
		Vec3<float> pos = getPos(i);

		// tangent along the strand
		Vec3<float> tangent;
		if (i < m_Count - 1) tangent = getPos(i + 1) - pos;
		else                 tangent = pos - getPos(i - 1);
		tangent.Normalize();

		// outward from the head, so the card lies flat against it
		Vec3<float> outward = pos - m_HeadCenter;
		if (outward.LengthSquared() < 1e-8f) outward.Set(0.0f, 0.0f, 1.0f);
		outward.Normalize();

		Vec3<float> width = tangent.Cross(outward);
		if (width.LengthSquared() < 1e-8f)
		{
			Vec3<float> fallback(1.0f, 0.0f, 0.0f);
			width = tangent.Cross(fallback);
		}
		width.Normalize();

		float taper = 1.0f - (float)i / (float)(m_Count - 1);	// 1 at root, 0 at tip
		float halfW = 0.8f * rootWidth * taper;

		// brown, a little lighter at the tip + slight per-strand variation
		float jitter = 0.5f * (m_InitPos.GetX() - (float)(int)m_InitPos.GetX());
		float shade = 0.30f + 0.20f * (1.0f - taper) + 0.05f * jitter;
		glColor3f(shade * 1.0f, shade * 0.7f, shade * 0.45f);

		Vec3<float> a = pos - width * halfW;
		Vec3<float> b = pos + width * halfW;
		glVertex3f(a.GetX(), a.GetY(), a.GetZ());
		glVertex3f(b.GetX(), b.GetY(), b.GetZ());
	}
	glEnd();
	glPopAttrib();
}
void stbLoadTexture(GLuint* tex, const char* filename, int req_channels) {
	// req_channels: 0 = BMP, 4 = JPEG
	int width = 0, height = 0, channels = 0;

	stbi_set_flip_vertically_on_load(true);
	stbi_uc* image = stbi_load(filename, &width, &height, &channels, req_channels);

	if (image != nullptr) {
		glGenTextures(1, tex);
		glBindTexture(GL_TEXTURE_2D, *tex);

		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_RGBA, GL_UNSIGNED_BYTE, image);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

		stbi_image_free(image);
	}
	else {
		cout << "ERROR: Can't load a texture!" << endl;
	}
}
void hairstrand::Draw()
{
	DrawLine();
}

void hairstrand::ApplyWind(Vec3<float> wind)
{
	for (int i = 0; i < m_Count - 1; i++)
	{
		if (m_InvMass[i] != 0.0f)
			m_Vel[i] += wind;
		ApplyWindForRope(wind, i, i + 1);
	}
}

void hairstrand::ApplyWindForRope(Vec3<float> wind, int index0, int index1)
{
	auto p0 = m_NewPos[index0];
	auto p1 = m_NewPos[index1];

	Vec3<float> dir = p1 - p0;
	Vec3<float> normal = Vec3<float>(dir.GetY(), -dir.GetX(), 0.0f);

	Vec3<float> force = normal * (normal.Dot(wind));

	if (m_InvMass[index0] != 0.0f)
		m_Vel[index0] += force;
	if (m_InvMass[index1] != 0.0f)
		m_Vel[index1] += force;
}
